#include "fibre/posix_udp.hpp"
#include <sys/types.h>
#include <QDebug>
//#pragma comment(lib,"Ws2_32.lib ")

void hexdump(const uint8_t* buf, size_t len) {
    for (size_t pos = 0; pos < len; ++pos) {
        printf(" %02x", buf[pos]);
        if ((((pos + 1) % 16) == 0) || ((pos + 1) == len))
            printf("\r\n");
    }
}

void fibre_udp_socket::client_udp_init(unsigned int port)
{
    WSAData wsa;
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        qDebug() << "error";
    }

    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        qDebug() << "error";
    }

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr= inet_addr("127.0.0.1");
}

int fibre_udp_socket::client_udp_send(const uint8_t* buffer, int length)
{
    int status = sendto(s, reinterpret_cast<const char *>(buffer), length, 0, reinterpret_cast<struct sockaddr*>(&si_me), sizeof(si_me));
    return status;
}

int fibre_udp_socket::client_udp_recv(uint8_t* buffer, int length)
{
    int len = sizeof(si_me);
    int n_received = recvfrom(s, reinterpret_cast<char *>(buffer), length, 0, reinterpret_cast<struct sockaddr*>(&si_me), &len);
    return n_received;
}

void fibre_udp_socket::client_udp_close()
{
    closesocket(s);
    WSACleanup();
}

bool fibre_udp_socket::UDP_transaction(const uint8_t* tx_buffer, int tx_length, uint8_t *rx_buffer, int rx_length)
{
    int n_received = 0;

    //hexdump(tx_buffer, tx_length);

    if(client_udp_send(tx_buffer, tx_length))
    {
        n_received = client_udp_recv(rx_buffer, rx_length);
        //hexdump(rx_buffer, rx_length);
    }

    return n_received == rx_length ? true : false;
}
