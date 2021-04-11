#include <sys/types.h>
#if 0
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#include <WinSock2.h>
#endif
#include <thread>
#include <future>
#include <vector>

#include <fibre/protocol.hpp>
#pragma comment(lib,"Ws2_32.lib ")

#define TCP_RX_BUF_LEN	512

class TCPStreamSink : public StreamSink {
public:
    TCPStreamSink(int socket_fd) :
        socket_fd_(socket_fd)
    {}

    int process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes) {
        int bytes_sent = send(socket_fd_, reinterpret_cast<const char *>(buffer), length, 0);
        if (processed_bytes)
            *processed_bytes = (bytes_sent == -1) ? 0 : bytes_sent;
        return (bytes_sent == -1) ? -1 : 0;
    }

    size_t get_free_space() { return SIZE_MAX; }

private:
    int socket_fd_;
};


int serve_client(int sock_fd) {
    uint8_t buf[TCP_RX_BUF_LEN];

    // initialize output stack for this client
    TCPStreamSink tcp_packet_output(sock_fd);
    StreamBasedPacketSink packet2stream(tcp_packet_output);
    BidirectionalPacketBasedChannel channel(packet2stream);

    StreamToPacketSegmenter stream2packet(channel);

    // now listen for it
    for (;;) {
        memset(buf, 0, sizeof(buf));
        // returns as soon as there is some data
        int n_received = recv(sock_fd, reinterpret_cast<char *>(buf), sizeof(buf), 0);

        // -1 indicates error and 0 means that the client gracefully terminated
        if (n_received == -1 || n_received == 0) {
            closesocket(sock_fd);
            return n_received;
        }

        // input processing stack
        size_t processed = 0;
        stream2packet.process_bytes(buf, n_received, &processed);
    }
}

// function to check if a worker thread handling a single client is done
template<typename T>
bool future_is_ready(std::future<T>& t){
    return t.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

int serve_on_tcp(unsigned int port) {
    int s;

    WSAData wsa;
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        return 0;
    }

    sockaddr_in service;
    sockaddr_in si_other;
    memset((char *) &service, 0, sizeof(service));
    service.sin_family = AF_INET;
    service.sin_addr.s_addr = INADDR_ANY;
    service.sin_port = htons(port);

    if ((s=socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
        return -1;
    }

    if (bind(s, reinterpret_cast<struct sockaddr *>(&service), sizeof(service)) == -1) {
        return -1;
    }

    listen(s, 128); // make this socket a passive socket
    std::vector<std::future<int>> serv_pool;
    for (;;) {
        memset(&si_other, 0, sizeof(si_other));

        int silen = sizeof(si_other);
        // TODO: Add a limit on accepting connections
        int client_portal_fd = accept(s, reinterpret_cast<sockaddr *>(&si_other), &silen); // blocking call
        serv_pool.push_back(std::async(std::launch::async, serve_client, client_portal_fd));
        // do a little clean up on the pool
        for (std::vector<std::future<int>>::iterator it = serv_pool.end()-1; it >= serv_pool.begin(); --it) {
            if (future_is_ready(*it)) {
                // we can erase this thread
                serv_pool.erase(it);
            }
        }
    }

    closesocket(s);
    WSACleanup();
}

