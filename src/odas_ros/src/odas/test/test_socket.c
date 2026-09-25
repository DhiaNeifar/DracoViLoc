/**
 * \file     test_socket.c
 * \brief    Unit tests for socket_send_all(). The send() libc call is
 *           replaced at link time with a scripted mock using
 *           -Wl,--wrap=send.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>

#include <utils/socket.h>

#define MOCK_MAX_CALLS 16

#ifndef MSG_NOSIGNAL
#error "MSG_NOSIGNAL is required"
#endif

static struct {

    ssize_t results[MOCK_MAX_CALLS];
    int errnos[MOCK_MAX_CALLS];
    int nCalls;

    int callIndex;
    int flags[MOCK_MAX_CALLS];
    size_t lens[MOCK_MAX_CALLS];

    unsigned char sink[8192];
    size_t sinkOffset;

} mock;

ssize_t __wrap_send(int sockfd, const void * buf, size_t len, int flags) {

    int idx = mock.callIndex;
    ssize_t result;

    (void) sockfd;

    if (idx >= MOCK_MAX_CALLS) {

        fprintf(stderr, "FAIL: mock received more calls than scripted\n");
        exit(EXIT_FAILURE);

    }

    mock.flags[idx] = flags;
    mock.lens[idx] = len;
    mock.callIndex++;

    if (idx >= mock.nCalls) {

        fprintf(stderr, "FAIL: mock received more calls than scripted\n");
        exit(EXIT_FAILURE);

    }

    result = mock.results[idx];

    if (result > 0) {

        memcpy(&(mock.sink[mock.sinkOffset]), buf, result);
        mock.sinkOffset += result;

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

static void script(const ssize_t * results, const int * errnos, int nCalls) {

    int iCall;

    for (iCall = 0; iCall < nCalls; iCall++) {

        mock.results[iCall] = results[iCall];
        mock.errnos[iCall] = errnos[iCall];

    }

    mock.nCalls = nCalls;

}

static void check_msgsignal(void) {

    int iCall;

    for (iCall = 0; iCall < mock.callIndex; iCall++) {

        CHECK((mock.flags[iCall] & MSG_NOSIGNAL) != 0);

    }

}

static void test_partial_sends(void) {

    unsigned char buffer[4096];
    ssize_t results[] = { 1000, 1000, 1000, 1096 };
    int errnos[] = { 0, 0, 0, 0 };
    int iByte;
    int rtnValue;

    mock_reset();
    for (iByte = 0; iByte < 4096; iByte++) {

        buffer[iByte] = (unsigned char) (iByte % 251);

    }
    script(results, errnos, 4);

    rtnValue = socket_send_all(7, (char *) buffer, 4096);

    CHECK(rtnValue == 0);
    CHECK(mock.callIndex == 4);
    CHECK(mock.sinkOffset == 4096);
    CHECK(memcmp(mock.sink, buffer, 4096) == 0);

    check_msgsignal();

}

static void test_eintr_retry(void) {

    unsigned char buffer[4096];
    ssize_t results[] = { -1, 1000, -1, 3096 };
    int errnos[] = { EINTR, 0, EINTR, 0 };
    int rtnValue;

    mock_reset();
    memset(buffer, 0x5A, sizeof(buffer));
    script(results, errnos, 4);

    rtnValue = socket_send_all(7, (char *) buffer, 4096);

    CHECK(rtnValue == 0);
    CHECK(mock.callIndex == 4);
    CHECK(mock.sinkOffset == 4096);
    CHECK(memcmp(mock.sink, buffer, 4096) == 0);

    check_msgsignal();

}

static void test_zero_return(void) {

    unsigned char buffer[1024];
    ssize_t results[] = { 0 };
    int errnos[] = { 0 };
    int rtnValue;

    mock_reset();
    memset(buffer, 0x5A, sizeof(buffer));
    script(results, errnos, 1);

    rtnValue = socket_send_all(7, (char *) buffer, 1024);

    CHECK(rtnValue == -1);
    CHECK(mock.callIndex == 1);

    check_msgsignal();

}

static void test_epipe(void) {

    unsigned char buffer[1024];
    ssize_t results[] = { -1 };
    int errnos[] = { EPIPE };
    int rtnValue;

    mock_reset();
    memset(buffer, 0x5A, sizeof(buffer));
    script(results, errnos, 1);

    rtnValue = socket_send_all(7, (char *) buffer, 1024);

    CHECK(rtnValue == -1);
    CHECK(mock.callIndex == 1);

    check_msgsignal();

}

static void test_econnreset(void) {

    unsigned char buffer[1024];
    ssize_t results[] = { -1 };
    int errnos[] = { ECONNRESET };
    int rtnValue;

    mock_reset();
    memset(buffer, 0x5A, sizeof(buffer));
    script(results, errnos, 1);

    rtnValue = socket_send_all(7, (char *) buffer, 1024);

    CHECK(rtnValue == -1);
    CHECK(mock.callIndex == 1);

    check_msgsignal();

}

static void test_partial_then_error(void) {

    unsigned char buffer[4096];
    ssize_t results[] = { 1000, -1 };
    int errnos[] = { 0, EPIPE };
    int rtnValue;

    mock_reset();
    memset(buffer, 0x5A, sizeof(buffer));
    script(results, errnos, 2);

    rtnValue = socket_send_all(7, (char *) buffer, 4096);

    CHECK(rtnValue == -1);
    CHECK(mock.callIndex == 2);
    CHECK(mock.sinkOffset == 1000);

    check_msgsignal();

}

static void test_empty_buffer(void) {

    int rtnValue;

    mock_reset();

    rtnValue = socket_send_all(7, NULL, 0);

    CHECK(rtnValue == 0);
    CHECK(mock.callIndex == 0);

}

int main(void) {

    test_partial_sends();
    test_eintr_retry();
    test_zero_return();
    test_epipe();
    test_econnreset();
    test_partial_then_error();
    test_empty_buffer();

    printf("PASS: test_socket\n");

    return EXIT_SUCCESS;

}
