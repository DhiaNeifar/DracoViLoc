/**
 * \file     test_src_hops.c
 * \brief    Unit tests for the full-hop ALSA accumulation logic in
 *           src_hops_process_interface_soundcard(). The snd_pcm_readi()
 *           ALSA call is replaced at link time with a scripted mock using
 *           -Wl,--wrap=snd_pcm_readi.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <alsa/asoundlib.h>

#include <source/src_hops.h>

#define MOCK_MAX_CALLS 16
#define CANARY_SIZE 64
#define CANARY_BYTE 0xEE

static unsigned int mock_nChannels = 2;
static unsigned int mock_bytesPerSample = 4;

static struct {

    snd_pcm_sframes_t results[MOCK_MAX_CALLS];
    int errnos[MOCK_MAX_CALLS];
    unsigned char fills[MOCK_MAX_CALLS];
    int nCalls;

    int callIndex;
    void * ptrs[MOCK_MAX_CALLS];
    snd_pcm_uframes_t sizes[MOCK_MAX_CALLS];

} mock;

snd_pcm_sframes_t __wrap_snd_pcm_readi(snd_pcm_t * pcm, void * buffer, snd_pcm_uframes_t size) {

    int idx = mock.callIndex;
    snd_pcm_sframes_t result;

    (void) pcm;

    if (idx >= MOCK_MAX_CALLS) {

        fprintf(stderr, "FAIL: mock received more calls than scripted\n");
        exit(EXIT_FAILURE);

    }

    mock.ptrs[idx] = buffer;
    mock.sizes[idx] = size;
    mock.callIndex++;

    result = mock.results[idx];

    if (result > 0) {

        memset(buffer, mock.fills[idx], result * mock_nChannels * mock_bytesPerSample);

    }
    else if (result < 0) {

        errno = mock.errnos[idx];

    }

    return result;

}

static void mock_reset(void) {

    memset(&mock, 0x00, sizeof(mock));

}

#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            fprintf(stderr, "FAIL: %s (line %d)\n", #cond, __LINE__); \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

static void check_canary(const unsigned char * buffer, unsigned int bufferSize) {

    unsigned int iByte;

    for (iByte = 0; iByte < CANARY_SIZE; iByte++) {

        CHECK(buffer[bufferSize + iByte] == CANARY_BYTE);

    }

}

static unsigned int fill_byte(unsigned int iCall) {

    return 0x10 + iCall;

}

static void setup_obj(src_hops_obj * obj, unsigned char ** buffer, unsigned int * bufferSize) {

    *bufferSize = 512 * 2 * 4;
    *buffer = malloc(*bufferSize + CANARY_SIZE);

    memset(*buffer, CANARY_BYTE, *bufferSize + CANARY_SIZE);
    memset(obj, 0x00, sizeof(src_hops_obj));

    obj->ch = (snd_pcm_t *) 0x1;
    obj->hopSize = 512;
    obj->nChannels = 2;
    obj->bufferSize = *bufferSize;
    obj->buffer = (char *) *buffer;

}

static void script(const snd_pcm_sframes_t * results, const int * errnos, int nCalls) {

    int iCall;

    for (iCall = 0; iCall < nCalls; iCall++) {

        mock.results[iCall] = results[iCall];
        mock.errnos[iCall] = errnos[iCall];
        mock.fills[iCall] = (unsigned char) fill_byte(iCall);

    }

    mock.nCalls = nCalls;

}

static void test_full_read(void) {

    src_hops_obj obj;
    unsigned char * buffer;
    unsigned int bufferSize;
    snd_pcm_sframes_t results[] = { 512 };
    int errnos[] = { 0 };
    int iByte;
    int rtnValue;

    mock_reset();
    setup_obj(&obj, &buffer, &bufferSize);
    script(results, errnos, 1);

    rtnValue = src_hops_process_interface_soundcard(&obj);

    CHECK(rtnValue == 0);
    CHECK(mock.callIndex == 1);
    CHECK(mock.ptrs[0] == buffer);
    CHECK(mock.sizes[0] == 512);

    for (iByte = 0; iByte < (int) bufferSize; iByte++) {

        CHECK(buffer[iByte] == fill_byte(0));

    }

    check_canary(buffer, bufferSize);

    free(buffer);

}

static void test_short_read_200_312(void) {

    src_hops_obj obj;
    unsigned char * buffer;
    unsigned int bufferSize;
    snd_pcm_sframes_t results[] = { 200, 312 };
    int errnos[] = { 0, 0 };
    int iByte;
    int rtnValue;

    mock_reset();
    setup_obj(&obj, &buffer, &bufferSize);
    script(results, errnos, 2);

    rtnValue = src_hops_process_interface_soundcard(&obj);

    CHECK(rtnValue == 0);
    CHECK(mock.callIndex == 2);
    CHECK(mock.ptrs[0] == buffer);
    CHECK(mock.sizes[0] == 512);
    CHECK(mock.ptrs[1] == buffer + 200 * 2 * 4);
    CHECK(mock.sizes[1] == 312);

    for (iByte = 0; iByte < 200 * 2 * 4; iByte++) {

        CHECK(buffer[iByte] == fill_byte(0));

    }

    for (iByte = 200 * 2 * 4; iByte < (int) bufferSize; iByte++) {

        CHECK(buffer[iByte] == fill_byte(1));

    }

    check_canary(buffer, bufferSize);

    free(buffer);

}

static void test_short_read_100_100_312(void) {

    src_hops_obj obj;
    unsigned char * buffer;
    unsigned int bufferSize;
    snd_pcm_sframes_t results[] = { 100, 100, 312 };
    int errnos[] = { 0, 0, 0 };
    int iByte;
    int rtnValue;

    mock_reset();
    setup_obj(&obj, &buffer, &bufferSize);
    script(results, errnos, 3);

    rtnValue = src_hops_process_interface_soundcard(&obj);

    CHECK(rtnValue == 0);
    CHECK(mock.callIndex == 3);
    CHECK(mock.ptrs[0] == buffer);
    CHECK(mock.sizes[0] == 512);
    CHECK(mock.ptrs[1] == buffer + 100 * 2 * 4);
    CHECK(mock.sizes[1] == 412);
    CHECK(mock.ptrs[2] == buffer + 200 * 2 * 4);
    CHECK(mock.sizes[2] == 312);

    for (iByte = 0; iByte < 100 * 2 * 4; iByte++) {

        CHECK(buffer[iByte] == fill_byte(0));

    }

    for (iByte = 100 * 2 * 4; iByte < 200 * 2 * 4; iByte++) {

        CHECK(buffer[iByte] == fill_byte(1));

    }

    for (iByte = 200 * 2 * 4; iByte < (int) bufferSize; iByte++) {

        CHECK(buffer[iByte] == fill_byte(2));

    }

    check_canary(buffer, bufferSize);

    free(buffer);

}

static void test_error_before_any_frame(void) {

    src_hops_obj obj;
    unsigned char * buffer;
    unsigned int bufferSize;
    snd_pcm_sframes_t results[] = { -32 };
    int errnos[] = { EPIPE };
    int iByte;
    int rtnValue;

    mock_reset();
    setup_obj(&obj, &buffer, &bufferSize);
    script(results, errnos, 1);

    rtnValue = src_hops_process_interface_soundcard(&obj);

    CHECK(rtnValue == -1);
    CHECK(mock.callIndex == 1);

    // No frame was captured: the buffer must remain untouched.

    for (iByte = 0; iByte < (int) bufferSize; iByte++) {

        CHECK(buffer[iByte] == CANARY_BYTE);

    }

    check_canary(buffer, bufferSize);

    free(buffer);

}

static void test_error_after_partial_read(void) {

    src_hops_obj obj;
    unsigned char * buffer;
    unsigned int bufferSize;
    snd_pcm_sframes_t results[] = { 200, -32 };
    int errnos[] = { 0, EPIPE };
    int iByte;
    int rtnValue;

    mock_reset();
    setup_obj(&obj, &buffer, &bufferSize);
    script(results, errnos, 2);

    rtnValue = src_hops_process_interface_soundcard(&obj);

    CHECK(rtnValue == -1);
    CHECK(mock.callIndex == 2);
    CHECK(mock.ptrs[1] == buffer + 200 * 2 * 4);
    CHECK(mock.sizes[1] == 312);

    // The 200 captured frames were written, but everything beyond the partial
    // read must remain untouched.

    for (iByte = 0; iByte < 200 * 2 * 4; iByte++) {

        CHECK(buffer[iByte] == fill_byte(0));

    }

    for (iByte = 200 * 2 * 4; iByte < (int) bufferSize; iByte++) {

        CHECK(buffer[iByte] == CANARY_BYTE);

    }

    check_canary(buffer, bufferSize);

    free(buffer);

}

static void test_zero_return(void) {

    src_hops_obj obj;
    unsigned char * buffer;
    unsigned int bufferSize;
    snd_pcm_sframes_t results[] = { 0 };
    int errnos[] = { 0 };
    int iByte;
    int rtnValue;

    mock_reset();
    setup_obj(&obj, &buffer, &bufferSize);
    script(results, errnos, 1);

    rtnValue = src_hops_process_interface_soundcard(&obj);

    CHECK(rtnValue == -1);
    CHECK(mock.callIndex == 1);

    // A zero return must not loop forever and must not touch the buffer.

    for (iByte = 0; iByte < (int) bufferSize; iByte++) {

        CHECK(buffer[iByte] == CANARY_BYTE);

    }

    check_canary(buffer, bufferSize);

    free(buffer);

}

int main(void) {

    test_full_read();
    test_short_read_200_312();
    test_short_read_100_100_312();
    test_error_before_any_frame();
    test_error_after_partial_read();
    test_zero_return();

    printf("PASS: test_src_hops\n");

    return EXIT_SUCCESS;

}
