#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <signal/demixing.h>
#include <signal/mask.h>
#include <signal/steer.h>
#include <signal/track.h>
#include <system/steer2demixing.h>

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL: %s (line %d)\n", #condition, __LINE__);     \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

static void fill_demixing(demixings_obj * demixings, float value)
{
    unsigned int i;
    const unsigned int count =
        demixings->halfFrameSize * demixings->nChannels * 2;

    for (i = 0; i < count; ++i) {
        demixings->array[0][i] = value;
    }
}

static void test_active_track_with_all_masks_disabled_is_fully_zeroed(void)
{
    const unsigned int nSeps = 1;
    const unsigned int nChannels = 3;
    const unsigned int halfFrameSize = 4;
    unsigned int i;
    steer2demixing_ds_obj * processor =
        steer2demixing_ds_construct_zero(nSeps, nChannels, halfFrameSize, 1.0e-20f);
    tracks_obj * tracks = tracks_construct_zero(nSeps);
    steers_obj * steers = steers_construct_zero(halfFrameSize, nSeps, nChannels);
    masks_obj * masks = masks_construct_zero(nSeps, nChannels);
    demixings_obj * demixings =
        demixings_construct_zero(halfFrameSize, nSeps, nChannels);

    tracks->ids[0] = 1;
    fill_demixing(demixings, 123.0f);

    steer2demixing_ds_process(processor, tracks, steers, masks, demixings);

    for (i = 0; i < halfFrameSize * nChannels * 2; ++i) {
        CHECK(demixings->array[0][i] == 0.0f);
    }

    demixings_destroy(demixings);
    masks_destroy(masks);
    steers_destroy(steers);
    tracks_destroy(tracks);
    steer2demixing_ds_destroy(processor);
}

static void test_disabled_channels_are_zeroed_for_every_bin(void)
{
    const unsigned int nSeps = 1;
    const unsigned int nChannels = 3;
    const unsigned int halfFrameSize = 4;
    unsigned int iBin;
    unsigned int iChannel;
    steer2demixing_ds_obj * processor =
        steer2demixing_ds_construct_zero(nSeps, nChannels, halfFrameSize, 1.0e-20f);
    tracks_obj * tracks = tracks_construct_zero(nSeps);
    steers_obj * steers = steers_construct_zero(halfFrameSize, nSeps, nChannels);
    masks_obj * masks = masks_construct_zero(nSeps, nChannels);
    demixings_obj * demixings =
        demixings_construct_zero(halfFrameSize, nSeps, nChannels);

    tracks->ids[0] = 1;
    masks->array[0] = 1;
    masks->array[1] = 0;
    masks->array[2] = 1;
    fill_demixing(demixings, 123.0f);

    for (iBin = 0; iBin < halfFrameSize; ++iBin) {
        for (iChannel = 0; iChannel < nChannels; ++iChannel) {
            const unsigned int index = (iBin * nChannels + iChannel) * 2;
            steers->array[0][index + 0] = 1.0f;
            steers->array[0][index + 1] = 0.0f;
        }
    }

    steer2demixing_ds_process(processor, tracks, steers, masks, demixings);

    for (iBin = 0; iBin < halfFrameSize; ++iBin) {
        for (iChannel = 0; iChannel < nChannels; ++iChannel) {
            const unsigned int index = (iBin * nChannels + iChannel) * 2;
            const float expected = iChannel == 1 ? 0.0f : 0.5f;
            CHECK(fabsf(demixings->array[0][index + 0] - expected) < 1.0e-6f);
            CHECK(demixings->array[0][index + 1] == 0.0f);
        }
    }

    demixings_destroy(demixings);
    masks_destroy(masks);
    steers_destroy(steers);
    tracks_destroy(tracks);
    steer2demixing_ds_destroy(processor);
}

int main(void)
{
    test_active_track_with_all_masks_disabled_is_fully_zeroed();
    test_disabled_channels_are_zeroed_for_every_bin();
    printf("PASS: test_steer2demixing\n");
    return EXIT_SUCCESS;
}
