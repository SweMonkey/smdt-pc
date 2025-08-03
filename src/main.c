#include <arpa/inet.h>  // inet_addr()
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>    // bzero()
#include <sys/socket.h>
#include <unistd.h>     // read(), write(), close()
#include <signal.h>
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <regex.h>

#define SA struct sockaddr
#define ADDR_LEN 256
#define BUF_SIZE 32

char gRemoteIP[ADDR_LEN];     // IP of server socket as ASCII string
uint16_t gRemotePort;         // Port of server socket
char gLocalAddress[ADDR_LEN]; // Address of local socket
int gRemote_fd = -1;          // Remote server socket
int gLocal_fd = -1;           // Local client socket/file
int bTTY_Connection = 0;      // 0 = Using sockets for remote - 1 = Using local serial port for remote
int bInMonitorMode = 0;       // Flag indicating if the emulated xport device is in monitor mode

// Logging files
FILE *pFileRemote, *pFileLocal;


void CloseRemoteSocket()
{
    if (gRemote_fd == -1) return;

    close(gRemote_fd);
    gRemote_fd = -1;

    strcpy(gRemoteIP, "0.0.0.0");
    gRemotePort = 0;

    uint8_t buf[4];
    snprintf(buf, 4, "D");
    write(gLocal_fd, buf, strlen(buf));
    
    printf("Remote socket closed.\n");
}

void Shutdown(void)
{
    printf("Shutting down...\n");

    // Close the sockets
    CloseRemoteSocket();
    close(gLocal_fd);

    if (pFileRemote != NULL) fclose(pFileRemote);
    if (pFileLocal != NULL) fclose(pFileLocal);
}

void SigCB(int signum)
{
    printf("Caught signal %d (%s)\n", signum, strsignal(signum));
    exit(signum);
}

void SigPipe(int signum)
{
    printf("Caught signal %d (%s)\n", signum, strsignal(signum));
    exit(signum);
    
}

int ResolveHost(char *hostname)
{
    struct hostent *hent;
    struct in_addr **addr_list;
    int i;

    if ((hent = gethostbyname(hostname)) == NULL)
    {
        printf("gethostbyname() error\n");
        return 1;
    }

    addr_list = (struct in_addr **)hent->h_addr_list;

    for (i = 0; addr_list[i] != NULL; i++)
    {
        strcpy(gRemoteIP, inet_ntoa(*addr_list[i]));
        return 0;
    }

    printf("ResolveHost() error\n");

    return 1;
}

int ConnectRemoteTCP()
{
    struct sockaddr_in servaddr, cli;

    if (gRemote_fd != -1) CloseRemoteSocket();

    // Socket create and verification
    gRemote_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (gRemote_fd == -1) 
    {
        printf("Socket creation failed...\n");
        return 1;
    }
    else printf("Socket successfully created...\n");

    bzero(&servaddr, sizeof(servaddr));

    // Assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(gRemoteIP);
    servaddr.sin_port = htons(gRemotePort);

    //struct timeval tv;
    //tv.tv_sec = 5;
    //tv.tv_usec = 0;
    //setsockopt(gRemote_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    //setsockopt(gRemote_fd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv);

    // Connect the client socket to server socket
    if (connect(gRemote_fd, (SA*)&servaddr, sizeof(servaddr)) != 0) 
    {
        printf("Connection to \"%s:%u\" failed...\n", gRemoteIP, gRemotePort);
        CloseRemoteSocket();

        return 2;
    }
    else printf("Connected to \"%s:%u\"...\n", gRemoteIP, gRemotePort);

    return 0;
}

void OpenLocalTTY()
{
    // Open the serial port
    gLocal_fd = open(gLocalAddress, O_RDWR);

    if (gLocal_fd < 0)
    {
        printf("Error %i from open(tty): %s\n", errno, strerror(errno));
        exit(EXIT_FAILURE); 
    }

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(gLocal_fd, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(EXIT_FAILURE); 
    }

    tty.c_cflag &= ~PARENB;         // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;         // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;          // Clear all bits that set the data size 
    tty.c_cflag |= CS8;             // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;           // Disable echo
    tty.c_lflag &= ~ECHOE;          // Disable erasure
    tty.c_lflag &= ~ECHONL;         // Disable new-line echo
    tty.c_lflag &= ~ISIG;           // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // Turn off s/w flow ctrl
    //tty.c_iflag |= (IXON | IXOFF | IXANY);    // Turn on s/w flow ctrl

    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);    // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;      // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;      // Prevent conversion of newline to carriage return/line feed
    //tty.c_oflag &= ~OXTABS;   // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    //tty.c_oflag &= ~ONOEOT;   // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 1;        // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 4800
    cfsetispeed(&tty, B4800);
    cfsetospeed(&tty, B4800);

    // Save tty settings, also checking for error
    if (tcsetattr(gLocal_fd, TCSANOW, &tty) != 0) 
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(EXIT_FAILURE); 
    }

    bTTY_Connection = 1;

    printf("Local tty \"%s\" is OK...\n", gLocalAddress);
}

int OpenLocalSocket()
{    
    struct sockaddr_un localsock;

    if (gLocal_fd != -1) 
    {
        close(gLocal_fd);
        gLocal_fd = -1;
    }
    
    // Create and verify local socket
    gLocal_fd = socket(AF_UNIX, SOCK_STREAM, 0);

    if (gLocal_fd == -1) 
    {
        perror("Local socket creation failed");
        exit(EXIT_FAILURE);
    }

    bzero(&localsock, sizeof(localsock));

    localsock.sun_family = AF_UNIX;    
    memcpy(localsock.sun_path, gLocalAddress, sizeof(localsock.sun_path));

    // Connect the client socket to server socket
    if (connect(gLocal_fd, (SA*)&localsock, sizeof(localsock)) != 0) 
    {
        int err = errno;

        perror("Local socket connection failed");

        if (err == ECONNREFUSED)
        {
            printf("Please delete the stale file \"%s\" and try again\n", gLocalAddress);
            exit(EXIT_FAILURE);
        }

        close(gLocal_fd);
        gLocal_fd = -1;
        return -1;
    }

    bTTY_Connection = 0;

    return 0;
}

int OpenRemoteSocket()
{    
    struct sockaddr_un remotesock;
    
    if (gRemote_fd != -1) 
    {
        close(gRemote_fd);
        gRemote_fd = -1;
    }
    
    // Create and verify remote socket
    gRemote_fd = socket(AF_UNIX, SOCK_STREAM, 0);

    if (gRemote_fd == -1) 
    {
        perror("Remote socket creation failed");
        exit(EXIT_FAILURE);
    }

    bzero(&remotesock, sizeof(remotesock));

    remotesock.sun_family = AF_UNIX;    
    memcpy(remotesock.sun_path, gRemoteIP, sizeof(remotesock.sun_path));

    // Connect the client socket to server socket
    if (connect(gRemote_fd, (SA*)&remotesock, sizeof(remotesock)) != 0) 
    {
        perror("Remote socket connection failed");
        close(gRemote_fd);
        gRemote_fd = -1;
        return -1;
    }

    return 0;
}

void RunRawTTY()
{
    OpenLocalTTY();
    if (ConnectRemoteTCP()) exit(EXIT_FAILURE);

    uint8_t buf_rx[BUF_SIZE], buf_tx[BUF_SIZE];
    int r_rx = 0, r_tx = 0;

    pFileRemote = fopen("rx.log" , "wb");
    pFileLocal = fopen("tx.log" , "wb");

    while(1)
    {
        r_rx = recv(gRemote_fd, &buf_rx, BUF_SIZE, MSG_DONTWAIT);

        if (r_rx)
        {
            write(gLocal_fd, buf_rx, r_rx);
            if (r_rx > 0) 
            {
                fwrite(buf_rx, sizeof(char), r_rx, pFileRemote);
                fflush(pFileRemote);
            }
        }

        r_tx = read(gLocal_fd, &buf_tx, BUF_SIZE);
        if (r_tx)
        {
            write(gRemote_fd, buf_tx, r_tx);
            if (r_tx > 0) 
            {
                fwrite(buf_tx, sizeof(char), r_tx, pFileLocal);
                fflush(pFileLocal);
            }
        }

        usleep(50);
    }

    exit(EXIT_SUCCESS);
}

void WaitForLocal()
{
    printf("Waiting for local connection on \"%s\"...\n", gLocalAddress);

    while (OpenLocalSocket() != 0)
    {
        fflush(stdout);
        sleep(5);
    }
    printf("\nLocal connected!\n");
}

void WaitForRemote()
{
    printf("Waiting for remote connection on \"%s\"...\n", gRemoteIP);

    while (OpenRemoteSocket() != 0)
    {
        fflush(stdout);
        sleep(5);
    }
    printf("\nRemote connected!\n");
}

uint8_t EnterMonitor()
{
    uint8_t buf[2] = {'0', '>'}; // "C0.0.0.0/0\n0>" -- 0> = OK, 9> = Invalid cmd

    printf("Entering monitor mode\n");
    fflush(stdout);

    if (gRemote_fd != -1)
    {
        printf("Connection is open, was the xPort reset? Closing remote socket...\n");
        CloseRemoteSocket();
    }

    write(gLocal_fd, buf, 2);

    bInMonitorMode = 1;

    return 1;
}

uint8_t ExitMonitor()
{
    uint8_t buf[6] = {'Q', 'U', '\n', '\r', '0', '>'}; // "QU\n0>" -- 0> = OK, 9> = Invalid cmd
    printf("Exiting monitor mode\n");   
    fflush(stdout);

    // In case the remote socket wasn't closed at EnterMonitor() for some reason...
    if (gRemote_fd != -1)
    {
        printf("Connection is open, was the xPort reset? Closing remote socket...\n");
        CloseRemoteSocket();
    }

    write(gLocal_fd, buf, 6);

    bInMonitorMode = 0;

    return 0;
}

void GetLocalIP()
{
    uint8_t buf[256];
    char ipbuf[64];

    // Read out "hostname -I" command output
    FILE *fd = popen("hostname -I", "r");

    if (fd == NULL) 
    {
        printf("Could not open pipe to \"hostname\"\n");
        snprintf(buf, 256, "0.0.0.0G");
    }
    else
    {
        // Put output into a string
        fgets(ipbuf, 64, fd);

        // Only keep the first ip.
        for (int i = 0; i < 64; ++i)
        {
            if (ipbuf[i] == ' ')
            {
                ipbuf[i] = '\0';
                break;
            }
        }

        printf("Returning IP address: %s\n", ipbuf);
        snprintf(buf, 256, "%sG", ipbuf);
    }
    
    fflush(stdout);
    write(gLocal_fd, buf, strlen(buf));
}

void PingIP(char *ipaddr)
{
    char cmdbuf[64];

    snprintf(cmdbuf, 64, "ping -c 5 -W 10 %s", ipaddr);

    char pingbuf[256];

    // Read out "ping" command output
    FILE *fd = popen(cmdbuf, "r");

    if (fd == NULL) 
    {
        printf("Could not open pipe to \"ping\"\n");
    }
    else
    {
        write(gLocal_fd, "  \r", 3);

        // Put output into a string
        for (int i = 0; i < 6; i++)
        {
            fgets(pingbuf, 256, fd);
            
            if (i == 0) continue;

            write(gLocal_fd, pingbuf, strlen(pingbuf));

            printf("Returning ping response: %.*s", strlen(pingbuf), pingbuf);

            if (i >= 5)
            {
                write(gLocal_fd, "\n", 1);
            }
        }
    }
    
    fflush(stdout);
}

typedef enum {
    PAT_NONE = 0,
    PAT_C_MONITOR,     // "C0.0.0.0/0"    - Enter monitor mode
    PAT_C_ADDR_PORT, // "Cxxxx:yyyy"    - Connect to address:port
    PAT_QU,          // "QU"            - Exit monitor mode
    PAT_NC,          // "NC"            - Get IP
    PAT_PI_IP        // "PI x.y.z.w"    - Ping IP
} PatternType;

PatternType XPort_get_pattern_type(const char* str) 
{
    regex_t regex;
    int ret;

    struct 
    {
        const char *pattern;
        PatternType type;
    } patterns[] = 
    {
        {"^C0\\.0\\.0\\.0/0$", PAT_C_MONITOR},
        {"^C[^:]+:[0-9]+$", PAT_C_ADDR_PORT},
        {"^QU$", PAT_QU},
        {"^NC$", PAT_NC},
        {"^PI [0-9]{1,3}(\\.[0-9]{1,3}){3}$", PAT_PI_IP}
    };

    for (int i = 0; i < sizeof(patterns)/sizeof(patterns[0]); i++) 
    {
        if (regcomp(&regex, patterns[i].pattern, REG_EXTENDED | REG_NOSUB) != 0)
            continue;

        ret = regexec(&regex, str, 0, NULL, 0);
        regfree(&regex);

        if (ret == 0)
            return patterns[i].type;
    }

    return PAT_NONE;
}

#define MAX_LINE_LEN 64

int XPort_process_tx_line(const char *line, int len) 
{
    char clean_line[MAX_LINE_LEN];

    memset(clean_line, 0, MAX_LINE_LEN);
    strncpy(clean_line, line, len);

    if (clean_line[len - 1] == '\n') 
    {
        clean_line[len - 1] = '\0';  // Trim newline
    }

    PatternType pat = XPort_get_pattern_type(clean_line);

    switch (pat) 
    {
        case PAT_C_MONITOR:
            EnterMonitor();
        break;

        case PAT_C_ADDR_PORT:
        {        
            char addr[128] = {0};
            int port = 0;
        
            // Match: C<addr>:<port>
            if (sscanf(clean_line, "C%127[^:]:%d", addr, &port) == 2) 
            {
                //printf("Addr: %s, Port: %d\n", addr, port);
                gRemotePort = port;

                if (ResolveHost(addr) == 0)
                {
                    char buf[4];
                    snprintf(buf, sizeof(buf), ConnectRemoteTCP() ? "N" : "C");

                    write(gLocal_fd, buf, strlen(buf));
                }
            } 
            else 
            {
                printf("Malformed connect address:port command\n");
            }
            break;
        }

        case PAT_QU:
            ExitMonitor();
        break;

        case PAT_NC:
            GetLocalIP();
        break;

        case PAT_PI_IP:
        {
            char ip_address[64] = {0};
        
            // Expecting "PI x.x.x.x"
            if (sscanf(clean_line, "PI %63s", ip_address) == 1) 
            {
                PingIP(ip_address);
            } 
            else 
            {
                printf("Malformed PI command\n");
            }
            break;
        }

        case PAT_NONE:
        default:
        return 0;
    }

    return 1;
}

void RunXPortEmulation()
{
    uint8_t buf_rx[BUF_SIZE], buf_tx[BUF_SIZE];   // Incoming/Outgoing byte buffer for recv
    int r_rx = 0, r_tx = 0;                       // Number of bytes received in buf_rx/buf_tx

    pFileRemote = fopen("rx.log", "wb");
    pFileLocal = fopen("tx.log", "wb");

    char rx_line_buf[MAX_LINE_LEN] = {0};
    int rx_line_len = 0;
    int bXPortMatch = 0;    // 0= None (Data) - 1= Match found, do not send it to remote

    while (1)
    {
        // Read from gRemote_fd if data is available
        r_rx = recv(gRemote_fd, buf_rx, sizeof(buf_rx), MSG_DONTWAIT);
        if (r_rx > 0) 
        {
            ssize_t n = 0;

            // Forward received data from gRemote_fd to gLocal_fd
            if (bTTY_Connection) 
                n = write(gLocal_fd, buf_rx, r_rx);
            else 
                n = send(gLocal_fd, buf_rx, r_rx, MSG_NOSIGNAL);

            if (n == -1) 
            {
                int err = errno;

                perror("write()/send() to local socket failed");

                if ((err == EPIPE) || (err == ECONNRESET)) 
                {  
                    CloseRemoteSocket();    // No point in keeping the remote side connected, close it...
                    WaitForLocal();         // Wait for local connection again
                }
            }

            // Log all RX traffic
            fwrite(buf_rx, sizeof(char), r_rx, pFileRemote);
            fflush(pFileRemote);
        }

        // Read from gLocal_fd if data is available
        if (bTTY_Connection) 
            r_tx = read(gLocal_fd, buf_tx, sizeof(buf_tx));
        else 
            r_tx = recv(gLocal_fd, buf_tx, sizeof(buf_tx), MSG_DONTWAIT);

        if (r_tx > 0)
        {
            fwrite(buf_tx, sizeof(char), r_tx, pFileLocal);
            fflush(pFileLocal);

            bXPortMatch = 0;
        
            for (int i = 0; i < r_tx && rx_line_len < MAX_LINE_LEN - 1; ++i) 
            {
                rx_line_buf[rx_line_len++] = buf_tx[i];

                // Check for newline
                if (buf_tx[i] == '\n') 
                {
                    bXPortMatch = XPort_process_tx_line(rx_line_buf, rx_line_len);
                    rx_line_len = 0;  // Reset for next message
                    memset(rx_line_buf, 0, MAX_LINE_LEN);
                }
            }

            if ((bXPortMatch == 0) && (gRemote_fd != -1))
            {
                ssize_t n = send(gRemote_fd, buf_tx, r_tx, MSG_WAITALL | MSG_NOSIGNAL);
                
                if (n == -1) 
                {
                    int err = errno;

                    perror("send() to remote socket failed");

                    if ((err == EPIPE) || (err == ECONNRESET)) 
                    {
                        CloseRemoteSocket();
                        rx_line_len = 0;
                    }
                }
                /*else if (n > 0)
                {
                    int EndLF = buf_tx[r_tx > 0 ? r_tx-1 : 0] == '\n' ? 1 : 0;
                    int EndCR = buf_tx[r_tx > 0 ? r_tx-1 : 0] == '\r' ? 1 : 0;
                    int r = EndLF + EndCR;

                    printf("Sending to remote: \"%.*s\" - ", r_tx-r, buf_tx);
                    printf("Ending in CR/LF: %s\n", r ? "yes" : "no ");
                }*/
                
                rx_line_len = 0;
                memset(rx_line_buf, 0, MAX_LINE_LEN);
            }
        }

        // Delay to avoid CPU overuse and to buffer data
        if (bTTY_Connection) usleep(2000);
        else usleep(50000);
    }

    exit(EXIT_SUCCESS);
}

void RunTEST()
{
    uint8_t buf_rx[BUF_SIZE], buf_tx[BUF_SIZE];   // Incoming/Outgoing byte buffer for recv
    int r_rx = 0, r_tx = 0;                       // Number of bytes received in buf_rx/buf_tx
    int len = 0;
    char c;

    while (1)
    {
        printf("Send a message: ");
        while (c != '\n')
        {
            c = getchar();
            buf_tx[len] = c;
            len++;
        }

        send(gLocal_fd, buf_tx, len, MSG_DONTWAIT);

        memset(buf_tx, 0, BUF_SIZE);

        // Read from gLocal_fd if data is available
        r_tx = recv(gLocal_fd, buf_tx, len, MSG_WAITALL);
        if (r_tx > 0)
        {
            printf("Message from local: %s\n", buf_tx);
        }

        len = 0;
        c = 0;

        // Small delay to avoid CPU overuse
        usleep(500);
    }

    exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
    signal(SIGINT, SigCB);
    signal(SIGTERM, SigCB);
    signal(SIGABRT, SigCB);
    signal(SIGFPE, SigCB);
    signal(SIGSEGV, SigCB);

    signal(SIGPIPE, SIG_IGN);
    //signal(SIGPIPE, SigPipe);

    #ifdef __linux__
    signal(SIGQUIT, SigCB);
    signal(SIGBUS, SigCB);
    #endif

    atexit(Shutdown);

    // xPort emulator communication using local serial port
    if ((argc > 2) && (strcmp(argv[1], "-xport") == 0))
    {
        strncpy(gLocalAddress, argv[2], ADDR_LEN);
        OpenLocalTTY();
        RunXPortEmulation();
    }
    // xPort emulator communication using sockets
    else if ((argc > 2) && (strcmp(argv[1], "-xportsock") == 0))
    {
        strncpy(gLocalAddress, argv[2], ADDR_LEN);        
        WaitForLocal();
        RunXPortEmulation();
    }
    // Test command for sending/receiving text
    else if ((argc > 2) && (strcmp(argv[1], "-test") == 0))
    {
        strncpy(gLocalAddress, argv[2], ADDR_LEN);
        WaitForLocal();
        RunTEST();
    }
    // Connect local serial port to remote server
    else if (argc == 3)
    {
        char TMP_Host[ADDR_LEN];

        if (sscanf(argv[1], "-local=%s", gLocalAddress) < 1)
        {
            printf("Error: No local tty specified!\n");
            exit(EXIT_FAILURE);
        }

        printf("Local: \"%s\"\n", gLocalAddress);

        if (sscanf(argv[2], "-remote=%s", TMP_Host) < 1)
        {
            printf("Error: No remote host ip:port specified!\n");
            exit(EXIT_FAILURE);
        }
        else
        {
            char *pch = strchr(TMP_Host, ':');

            if (pch != NULL)
            {
                gRemotePort = atoi(TMP_Host+(pch-TMP_Host+1));
                TMP_Host[pch-TMP_Host] = '\0';
            }
            else
            {
                gRemotePort = 0;
            }
        }

        if (ResolveHost(TMP_Host)) exit(EXIT_FAILURE);

        RunRawTTY();
    }
    else 
    {
        printf("SMDT PC Communicator v1.3\n\nUsage example:\n", argv[0]);
        printf("%s -local=/dev/ttyS4 -remote=127.0.0.1:6969     - Connect local serial port to remote server\n", argv[0]);
        printf("%s -xport /dev/ttyS4                            - xPort emulator communication using local serial port\n", argv[0]);
        printf("%s -xportsock ./socketfile.sock                 - xPort emulator communication using sockets\n\n", argv[0]);
    }

    exit(EXIT_SUCCESS);
}
