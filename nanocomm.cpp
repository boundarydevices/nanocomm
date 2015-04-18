/*
 * nanocomm.cpp
 *
 * minimal terminal emulator for testing serial ports.
 *
 * Usage:
 *	nanocomm /dev/ttymxc4 11500 [8 [N [1]]]
 *
 */

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/poll.h>
#include <ctype.h>
#include <stdlib.h>
#include <linux/serial.h>
#include <string.h>
#include <termios.h>

static unsigned const _standardBauds[] = {
	0,
	50,
	75,
	110,
	134,
	150,
	200,
	300,
	600,
	1200,
	1800,
	2400,
	4800,
	9600,
	19200,
	38400
};
static unsigned const numStandardBauds = sizeof(_standardBauds) / sizeof(_standardBauds[0]);

static unsigned const _highSpeedMask = 0010000 ;
static unsigned const _highSpeedBauds[] = {
	0,
	57600,
	115200,
	230400,
	460800,
	500000,
	576000,
	921600,
	1000000,
	1152000,
	1500000,
	2000000,
	2500000,
	3000000,
	3500000,
	4000000
};

static unsigned const numHighSpeedBauds = sizeof(_highSpeedBauds)/sizeof(_highSpeedBauds[0]);
bool baudRateToConst(unsigned bps, unsigned &constant)
{
	unsigned baudIdx = 0 ;
	bool haveBaud = false ;

	unsigned i ;
	for (i = 0 ; i < numStandardBauds ; i++) {
		if (_standardBauds[i] == bps) {
			haveBaud = true ;
			baudIdx = i ;
			break;
		}
	}

	if (!haveBaud) {
		for (i = 0 ; i < numHighSpeedBauds ; i++) {
			if (_highSpeedBauds[i] == bps) {
				haveBaud = true ;
				baudIdx = i | _highSpeedMask ;
				break;
			}
		}
	}

	constant = baudIdx ;
	return haveBaud ;
}

static bool volatile doExit = false ;
static void ctrlcHandler(int signo)
{
	printf("<ctrl-c>\r\n");
	doExit = true ;
}

static void setRaw(int fd,
		    int baud,
		    int databits,
		    char parity,
		    unsigned stopBits,
		    struct termios &oldState)
{
	tcgetattr(fd,&oldState);

	/* set raw mode for keyboard input */
	struct termios newState = oldState;
	newState.c_cc[VMIN] = 1;

	bool nonStandard = false ;
	unsigned baudConst ;
	if (baudRateToConst(baud, baudConst)) {
		cfsetispeed(&newState, baudConst);
		cfsetospeed(&newState, baudConst);
	}
	else {
		cfsetispeed(&newState,B38400);
		cfsetospeed(&newState,B38400);
		bool worked = false ;
		struct serial_struct nuts;
		int rval = ioctl(fd, TIOCGSERIAL, &nuts);
		if (0 == rval) {
			unsigned const divisor = nuts.baud_base / baud ;
			nuts.custom_divisor = divisor ;
			nuts.flags &= ~ASYNC_SPD_MASK;
			nuts.flags |= ASYNC_SPD_CUST;
			rval = ioctl(fd, TIOCSSERIAL, &nuts);
			if (0 == rval) {
				printf("baud changed\n");
				rval = ioctl(fd, TIOCGSERIAL, &nuts);
				if (0 == rval) {
					printf("divisor is now %u\n", nuts.custom_divisor);
				}
				else
					perror("TIOCGSERIAL2");
			}
			else
				perror("TIOCSSERIAL");
		}
		else
			perror("TIOCGSERIAL");
	} // non-standard serial

	//
	// Note that this doesn't appear to work!
	// Reads always seem to be terminated at 16 chars!
	//
	newState.c_cc[VTIME] = 0; // 1/10th's of a second, see http://www.opengroup.org/onlinepubs/007908799/xbd/termios.html

	newState.c_cflag &= ~(PARENB|CSTOPB|CSIZE|CRTSCTS);		 // Mask character size to 8 bits, no parity, Disable hardware flow control

	if ('E' == parity) {
		newState.c_cflag |= PARENB ;
		newState.c_cflag &= ~PARODD ;
	}
	else if ('O' == parity) {
		newState.c_cflag |= PARENB | PARODD ;
	}
	else if ('S' == parity) {
		newState.c_cflag |= PARENB | IGNPAR | CMSPAR ;
		newState.c_cflag &= ~PARODD ;
	}
	else if ('M' == parity) {
		newState.c_cflag |= PARENB | IGNPAR | CMSPAR | PARODD ;
	}
	else {
	} // no parity... already set

	newState.c_cflag |= (CLOCAL | CREAD |CS8);			 // Select 8 data bits
	if (7 == databits) {
		newState.c_cflag &= ~CS8 ;
	}

	if (1 != stopBits)
		newState.c_cflag |= CSTOPB ;

	newState.c_lflag &= ~(ICANON | ECHO);				 // set raw mode for input
	newState.c_iflag &= ~(IXON | IXOFF | IXANY|INLCR|ICRNL|IUCLC);	 //no software flow control
	newState.c_oflag &= ~OPOST;			 //raw output
	tcsetattr(fd, TCSANOW, &newState);
}

int main(int argc, char const * const argv[])
{
	if (2 >= argc) {
		fprintf(stderr,
			"Usage: %s device baud "
				"[databits=8 [parity=N [stopbits=1]]]\n",
			argv[0]);
		return -1;
	}
        char const *const deviceName = argv[1];
        int const fdSerial = open(deviceName, O_RDWR);
        if (0 > fdSerial) {
		perror(deviceName);
		return -1;
	}
	fcntl(fdSerial, F_SETFD, FD_CLOEXEC);
	fcntl(fdSerial, F_SETFL, O_NONBLOCK);

	int baud = strtoul(argv[2], 0, 0);
	int databits = ((3 < argc) && (7 == strtoul(argv[3], 0, 0)))
		       ? 7
		       : 8 ;
	char parity = (4 < argc)
		      ? toupper(*argv[4])
		      : 'N' ;
	unsigned stopBits = ((5 < argc) && ('2' == *argv[5]))
			    ? 2
			    : 1 ;

	printf("device %s opened: %u baud, %d bits, parity %c\n", deviceName, baud, databits, parity);
	struct termios oldSerialState;
	setRaw(fdSerial, baud, databits, parity, stopBits, oldSerialState);

	struct termios oldStdinState;
	tcgetattr(0,&oldStdinState);
	/* set raw mode for keyboard input */
	struct termios newState = oldStdinState;
	newState.c_cc[VMIN] = 1;


	newState.c_lflag &= ~(ICANON | ECHO);				 // set raw mode for input
	newState.c_iflag &= ~(IXON | IXOFF | IXANY|INLCR|ICRNL|IUCLC);	 //no software flow control
	newState.c_oflag &= ~OPOST;			 //raw output
	tcsetattr(0, TCSANOW, &newState);

	signal(SIGINT, ctrlcHandler);
	pollfd fds[2];
	fds[0].fd = fdSerial ;
	fds[0].events = POLLIN | POLLERR ;
	fds[1].fd = fileno(stdin);
	fds[1].events = POLLIN | POLLERR ;

	while (!doExit) {
		::poll(fds, 2, 1000);
		for (unsigned i = 0 ; i < 2 ; i++) {
                        if (fds[i].revents & POLLIN) {
				char inBuf[80];
				int numRead = read(fds[i].fd, inBuf, sizeof(inBuf));
				if (0 < numRead) {
					int fdout = i ? fdSerial : 1;
					write(fdout,inBuf,numRead);
				}
			}
		}
	}

	tcsetattr(fdSerial, TCSANOW, &oldSerialState);
	tcsetattr(0, TCSANOW, &oldStdinState);

	close(fdSerial);
	return 0 ;
}
