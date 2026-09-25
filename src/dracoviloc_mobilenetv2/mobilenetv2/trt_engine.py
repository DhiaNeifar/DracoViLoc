"""TensorRT 10 runner: raw float32 waveform [16000] -> probabilities [2]."""
from pathlib import Path

import numpy as np


class TrtEngine:
    def __init__(self, engine_path):
        # Lazy imports let CPU tests and --help run without CUDA or PyTorch.
        import pycuda.driver as cuda
        import tensorrt as trt

        self.cuda = cuda
        self.device_buffers = {}
        self.cuda_context = None
        path = Path(engine_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'MobileNetV2 engine not found: {path}; '
                                    'set engine_path or build the FP32 engine')
        cuda.init()
        self.cuda_context = cuda.Device(0).make_context()
        try:
            self.logger = trt.Logger(trt.Logger.WARNING)
            self.runtime = trt.Runtime(self.logger)
            self.engine = self.runtime.deserialize_cuda_engine(path.read_bytes())
            if self.engine is None:
                raise RuntimeError('Cannot deserialize MobileNetV2 engine; rebuild on this Jetson')
            self.context = self.engine.create_execution_context()
            if self.context is None:
                raise RuntimeError('Cannot create MobileNetV2 execution context')
            names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
            inputs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
            outputs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT]
            if inputs != ['waveform'] or outputs != ['probabilities']:
                raise ValueError(f'Unexpected model bindings: inputs={inputs}, outputs={outputs}')
            if not self.context.set_input_shape('waveform', (1, 16000)):
                raise ValueError('Engine does not accept waveform [1, 16000]')
            self.host_buffers = {}
            for name, shape in [('waveform', (1, 16000)), ('probabilities', (1, 2))]:
                if (self.engine.get_tensor_dtype(name) != trt.float32
                        or tuple(self.context.get_tensor_shape(name)) != shape):
                    raise ValueError(f'{name} must be float32 {shape}')
                self.host_buffers[name] = cuda.pagelocked_empty(shape, dtype=np.float32)
                self.device_buffers[name] = cuda.mem_alloc(self.host_buffers[name].nbytes)
                if not self.context.set_tensor_address(name, int(self.device_buffers[name])):
                    raise RuntimeError(f'Cannot bind {name}')
            self.stream = cuda.Stream()
        except BaseException:
            self._release()
            raise
        finally:
            cuda.Context.pop()
            # Failed initialization still needs to release the driver context.
            if getattr(self, 'engine', None) is None:
                self.cuda_context.detach()
                self.cuda_context = None

    def infer(self, waveform):
        waveform = np.asarray(waveform, dtype=np.float32)
        if waveform.shape != (16000,) or not np.isfinite(waveform).all():
            raise ValueError('Expected a finite float32 waveform [16000]')
        if self.cuda_context is None:
            raise RuntimeError('MobileNetV2 engine is closed')
        self.cuda_context.push()
        try:
            self.host_buffers['waveform'][0] = waveform
            self.cuda.memcpy_htod_async(self.device_buffers['waveform'],
                                       self.host_buffers['waveform'], self.stream)
            if not self.context.execute_async_v3(self.stream.handle):
                raise RuntimeError('MobileNetV2 TensorRT execution failed')
            self.cuda.memcpy_dtoh_async(self.host_buffers['probabilities'],
                                       self.device_buffers['probabilities'], self.stream)
            self.stream.synchronize()
            probabilities = self.host_buffers['probabilities'][0].copy()
            if (not np.isfinite(probabilities).all() or np.any(probabilities < 0)
                    or np.any(probabilities > 1)
                    or not np.isclose(probabilities.sum(), 1.0, atol=1e-3)):
                raise RuntimeError(f'Invalid model probabilities: {probabilities}')
            return probabilities
        finally:
            self.cuda.Context.pop()

    def _release(self):
        if getattr(self, 'stream', None) is not None:
            self.stream.synchronize()
        self.context = None
        for allocation in self.device_buffers.values():
            allocation.free()
        self.device_buffers.clear()
        self.host_buffers = {}
        self.stream = None
        self.engine = None
        self.runtime = None

    def close(self):
        if self.cuda_context is not None:
            self.cuda_context.push()
            try:
                self._release()
            finally:
                self.cuda.Context.pop()
                self.cuda_context.detach()
                self.cuda_context = None
