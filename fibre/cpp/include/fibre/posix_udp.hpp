#ifndef POSIX_UDP_H
#define POSIX_UDP_H
#include <stdint.h>
#if 0
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#include <WinSock2.h>
#endif

class fibre_udp_socket{
public:
    fibre_udp_socket()
    {}
    void client_udp_init(unsigned int port);

    int client_udp_send(const uint8_t* buffer, int length);

    int client_udp_recv(uint8_t* buffer, int length);

    void client_udp_close();

    bool UDP_transaction(const uint8_t* tx_buffer, int tx_length, uint8_t* rx_buffer, int rx_length);
private:
    struct sockaddr_in si_me;
    int s;
};
#endif
