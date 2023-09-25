#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <cstring>
#include <iostream>

class UDPSocket
{
public:
  UDPSocket(int const port)
  {
    sock = open_socket();
    bind_socket(port);
  }
  ~UDPSocket()
  {
    close_socket();
  }

  ssize_t read_socket(uint8_t * buf, int buf_size)
  {
    if (!is_sock_bound) {
      return -1;
    }

    memset(buf, 0, buf_size);
    ssize_t const len = recv(sock, buf, buf_size, 0);

    if (len == -1) {
      // Failed to read socket.
    } else if (len == buf_size) {
      // Buffer is small.
    }
    return len;
  }

private:
  int sock = -1;
  bool is_sock_bound = false;

  bool is_sock_opened()
  {
    return !(sock == -1);
  }

  int open_socket()
  {
    int const sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1) {
      // Failed to open socket.
    }
    return sock;
  }

  bool bind_socket(int const port)
  {
    if (!is_sock_opened()) {
      is_sock_bound = false;
      return false;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr("0.0.0.0");

    if (bind(sock, (sockaddr *)&addr, sizeof(addr)) == -1) {
      // Failed to bind socket.
      is_sock_bound = false;
      return false;
    }
    is_sock_bound = true;
    return true;
  }

  void close_socket()
  {
    shutdown(sock, 2);
  }

};
