
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

#define SA struct sockaddr
#define ADDR_LEN 256

char gRemoteIP[ADDR_LEN];     // IP of server socket as ASCII string
uint16_t gRemotePort;         // Port of server socket
char gLocalAddress[ADDR_LEN]; // Address of local socket
int gRemote_fd = -1;          // Remote server socket
int gLocal_fd = -1;           // Local client socket/file
int bTTY_Connection = 0;

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

    fclose(pFileRemote);
    fclose(pFileLocal);
}

void SignCB(int signum)
{
    printf("Caught signal %u\n", signum);
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

    addr_list = (struct in_addr **) hent->h_addr_list;

    for(i = 0; addr_list[i] != NULL; i++)
    {
        strcpy(gRemoteIP , inet_ntoa(*addr_list[i]));
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

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    //tty.c_iflag |= (IXON | IXOFF | IXANY); // Turn on s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
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
    
    // Socket create and verification
    if (gLocal_fd == -1) gLocal_fd = socket(AF_UNIX, SOCK_STREAM, 0);

    if (gLocal_fd == -1) 
    {
        printf("Local socket connection failed...\n");
        exit(EXIT_FAILURE); 
    }

    bzero(&localsock, sizeof(localsock));

    localsock.sun_family = AF_UNIX;    
    memcpy(localsock.sun_path, gLocalAddress, sizeof(localsock.sun_path));

    // Connect the client socket to server socket
    if (connect(gLocal_fd, (SA*)&localsock, sizeof(localsock)) != 0) 
    {
        return -1;
    }

    bTTY_Connection = 0;

    return 0;
}

int OpenRemoteSocket()
{    
    struct sockaddr_un remotesock;
    
    // Socket create and verification
    if (gRemote_fd == -1) gRemote_fd = socket(AF_UNIX, SOCK_STREAM, 0);

    if (gRemote_fd == -1) 
    {
        printf("Remote socket connection failed...\n");
        exit(EXIT_FAILURE); 
    }

    bzero(&remotesock, sizeof(remotesock));

    remotesock.sun_family = AF_UNIX;    
    memcpy(remotesock.sun_path, gRemoteIP, sizeof(remotesock.sun_path));

    // Connect the client socket to server socket
    if (connect(gRemote_fd, (SA*)&remotesock, sizeof(remotesock)) != 0) 
    {
        return -1;
    }

    return 0;
}

void RunRawTTY()
{
    OpenLocalTTY();
    if (ConnectRemoteTCP()) exit(EXIT_FAILURE);

    uint8_t buf_rx[256], buf_tx[256];
    int r_rx = 0, r_tx = 0;

    pFileRemote = fopen("rx.log" , "wb");
    pFileLocal = fopen("tx.log" , "wb");

    while(1)
    {
        r_rx = recv(gRemote_fd, &buf_rx, 256, MSG_DONTWAIT);

        if (r_rx)
        {
            write(gLocal_fd, buf_rx, r_rx);
            if (r_rx > 0) 
            {
                fwrite(buf_rx, sizeof(char), r_rx, pFileRemote);
                fflush(pFileRemote);
            }
        }

        r_tx = read(gLocal_fd, &buf_tx, 256);
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
        sleep(3);
    }
}

void WaitForRemote()
{
    printf("Waiting for remote connection on \"%s\"...\n", gRemoteIP);

    while (OpenRemoteSocket() != 0)
    {
        sleep(3);
    }
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
    uint8_t buf[256];
    printf("Pinging IP address \"%s\" ...\n", ipaddr);   
    fflush(stdout);

    snprintf(buf, 256, "  Ping is not implemented in xPort emu\n"); // Note the leading 2 spaces, SMDTC ignores those and only prints the following text

    // Send this message 5 times, SMDTC only stops the pinging after receiving 5 newlines (\n)
    write(gLocal_fd, buf, strlen(buf));
    write(gLocal_fd, buf, strlen(buf));
    write(gLocal_fd, buf, strlen(buf));
    write(gLocal_fd, buf, strlen(buf));
    write(gLocal_fd, buf, strlen(buf));
}

void RunXPortEmulation()
{
    uint8_t buf_rx[256], buf_tx[256];   // Incoming/Outgoing byte buffer for recv
    int r_rx = 0, r_tx = 0;             // Number of bytes received in buf_rx/buf_tx
    int bInMonitorMode = 0;             // Flag indicating if the emulated xport device is in monitor mode

    // Enter monitor mode specific pattern matching variables (Used when connected to remote server)
    int EC = 0, NC = 0;
    const char ENTER_PATTERN[] = "C0.0.0.0/0";
    const int ENTER_LENGTH = sizeof(ENTER_PATTERN) - 1; // Length of pattern without null terminator

    // Buffer for gLocal_fd data - Used to accumulate and check for the target command
    uint8_t rxdata_buffer[512];
    int rxdata_buffer_len = 0;

    pFileRemote = fopen("rx.log", "wb");
    pFileLocal = fopen("tx.log", "wb");

    while (1)
    {
        // Read from gRemote_fd if data is available
        r_rx = recv(gRemote_fd, buf_rx, sizeof(buf_rx), MSG_DONTWAIT);
        if (r_rx > 0) 
        {
            // Forward received data from gRemote_fd to gLocal_fd
            write(gLocal_fd, buf_rx, r_rx);

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
            // Log all TX traffic
            fwrite(buf_tx, sizeof(char), r_tx, pFileLocal);
            fflush(pFileLocal);

            // If connected to remote server, handle data transfer
            if (gRemote_fd != -1)
            {
                // Append received data to the persistent buffer
                if (rxdata_buffer_len + r_tx < sizeof(rxdata_buffer))
                {
                    memcpy(rxdata_buffer + rxdata_buffer_len, buf_tx, r_tx);
                    rxdata_buffer_len += r_tx;

                    NC++; // Increment received character count

                    if (rxdata_buffer[NC - 1] == ENTER_PATTERN[EC]) 
                    {
                        EC++; // Increment expected count if this character matches the pattern
                    } 
                    else 
                    {
                        // Reset both counters if there's a mismatch
                        NC = 0;
                        EC = 0;
                    }

                    // If the entire pattern is matched, enter monitor mode
                    if (EC == ENTER_LENGTH) 
                    {
                        bInMonitorMode = EnterMonitor();
                        rxdata_buffer_len = 0;
                        EC = 0;
                        NC = 0;
                    } 
                    //else if (EC == 0) 
                    // ^ This is commented out because the above code makes the user unable to type/send any of the characters in ENTER_PATTERN
                    // This is a workaround to keep sending the characters even if it is meant only for the xport emulator...
                    {
                        // If we aren't in the middle of matching the pattern, forward data to gRemote_fd
                        send(gRemote_fd, rxdata_buffer, rxdata_buffer_len, MSG_DONTWAIT);
                        rxdata_buffer_len = 0; // Clear buffer after forwarding
                        NC = 0;
                    }
                }
            }
            else
            {
                // If gRemote_fd is not connected, just accumulate and parse for commands

                // Append received data to the persistent buffer
                if (rxdata_buffer_len + r_tx < sizeof(rxdata_buffer))
                {
                    memcpy(rxdata_buffer + rxdata_buffer_len, buf_tx, r_tx);
                    rxdata_buffer_len += r_tx;
                }

                // Process buffer only if it contains a newline character
                uint8_t *newline = memchr(rxdata_buffer, '\n', rxdata_buffer_len);
                if (newline)
                {
                    // Calculate the length of the command up to and including the newline
                    int cmd_len = newline - rxdata_buffer + 1;

                    // Temporarily null-terminate for command matching
                    rxdata_buffer[cmd_len - 1] = '\0';

                    // Check for known commands in order of specificity
                    if (strcmp((char *)rxdata_buffer, "QU") == 0)
                    {
                        bInMonitorMode = ExitMonitor();
                    }
                    else if (strcmp((char *)rxdata_buffer, "C0.0.0.0/0") == 0)
                    {
                        bInMonitorMode = EnterMonitor();
                    }
                    else if (bInMonitorMode && strcmp((char *)rxdata_buffer, "NC") == 0)
                    {
                        GetLocalIP();
                    }
                    else if (bInMonitorMode && strncmp((char *)rxdata_buffer, "PI ", 3) == 0)
                    {
                        // Process "PI <IP>" command
                        char ip_address[256] = {0};
                        strncpy(ip_address, (char *)rxdata_buffer + 3, cmd_len - 4); // Exclude "PI " and '\0'
                        PingIP(ip_address);
                    }
                    else if (rxdata_buffer[0] == 'C' && gRemote_fd == -1)
                    {
                        // Process "C<address>" command
                        char target[256] = {0};
                        strncpy(target, (char *)rxdata_buffer + 1, cmd_len - 2);  // Exclude 'C' and '\0'

                        // Extract port if specified
                        char *port_str = strchr(target, ':');
                        if (port_str)
                        {
                            *port_str = '\0';
                            port_str++;
                            gRemotePort = atoi(port_str);
                        }
                        else
                        {
                            gRemotePort = 0;
                        }

                        if (ResolveHost(target) == 0)
                        {
                            char buf[4];
                            snprintf(buf, sizeof(buf), ConnectRemoteTCP() ? "N" : "C");

                            write(gLocal_fd, buf, strlen(buf));
                        }
                    }

                    // Remove processed command from the buffer
                    rxdata_buffer_len -= cmd_len;
                    memmove(rxdata_buffer, rxdata_buffer + cmd_len, rxdata_buffer_len);
                }
            }
        }

        // Small delay to avoid CPU overuse
        usleep(500);
    }

    exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
    signal(SIGINT, SignCB);
    signal(SIGTERM, SignCB);
    signal(SIGABRT, SignCB);
    signal(SIGFPE, SignCB);
    signal(SIGSEGV, SignCB);

    #ifdef __linux__
    signal(SIGQUIT, SignCB);
    signal(SIGBUS, SignCB);
    #endif

    atexit(Shutdown);

    if ((argc > 2) && (strcmp(argv[1], "-xport") == 0))
    {
        strncpy(gLocalAddress, argv[2], ADDR_LEN);
        OpenLocalTTY();

        RunXPortEmulation();
    }
    else if ((argc > 2) && (strcmp(argv[1], "-xportsock") == 0))
    {
        strncpy(gLocalAddress, argv[2], ADDR_LEN);
        if (OpenLocalSocket())
        {
            WaitForLocal();
        }

        RunXPortEmulation();
    }
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
        printf("SMDT PC Communicator v1.2\n\nUsage example:\n", argv[0]);
        printf("%s -local=/dev/ttyS4 -remote=127.0.0.1:6969     - Connect local serial port to remote server\n", argv[0]);
        printf("%s -xport /dev/ttyS4                            - xPort emulator communication using local serial port\n", argv[0]);
        printf("%s -xportsock ./socketfile.sock                 - xPort emulator communication using sockets\n\n", argv[0]);
        exit(EXIT_SUCCESS);
    }

    exit(EXIT_SUCCESS);
}
