/* Copyright @ Isa Ansharullah of aLab inc. 2016 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <sys/time.h>
#include <signal.h>

#define DEBUG_PRINT

#ifdef DEBUG_PRINT
#define dbg(...) printf(__VA_ARGS__)
#else
#define dbg(...)
#endif

uint32_t rxfd; // file descriptor for our rx serial comm
uint32_t txfd; // file descriptor for our tx serial comm

#define RXPACKET 0
#define TXPACKET 1

#define DUMP_RAW 1 // dump the raw packet
#define DUMP_EXP 2 // dump the packet with explanation

static uint32_t packet_dump_type = DUMP_RAW;

#define BYTEBUFSIZE 512
#define WRITEBUFSIZE 512

uint8_t bytebuf[BYTEBUFSIZE];
uint16_t bytepos;
uint16_t bytetoread;

uint8_t writebuf[WRITEBUFSIZE];

typedef enum byte_state {
	BSTATE_IDLE = 0,
	BSTATE_HEADER,
	BSTATE_WAIT_LEN1,
	BSTATE_WAIT_LEN2,
	BSTATE_WAIT_BODY,
} bstate;

#define BLECOMM_PKT_HEADER  0x3A
#define BLECOMM_PKT_STOP  0x0A

bstate bytest = BSTATE_IDLE;

enum {
	POS_HEAD = 0,
	POS_LENGTH,
	POS_TYPE = 3,
	POS_SUBTYPE = 4,
	POS_PAYLOAD = 4,
	POS_ATT_CHRNUM = POS_SUBTYPE,
	POS_ATT_SEQ = POS_SUBTYPE+3,
	POS_ATT_CMD0 = POS_SUBTYPE+4,
	POS_ATT_CMD1 = POS_SUBTYPE+5,
};

#define TYPE_BLEATT 0xB2 // gatt related
#define TYPE_BLEMOD 0xB3 // command from here to ble
#define TYPE_BLECTL 0xB4 // command from ble to here

#define SUBTYPE_READY   0x02
#define SUBTYPE_DISCONN 0x01
#define SUBTYPE_CONNECT 0x00

#define ATT_CHAR_VEND_FNC 0x00
#define ATT_CHAR_VEND_INF 0x01
#define ATT_CHAR_VEND_CMD 0x05
#define ATT_CHAR_VEND_RSP 0x06
#define ATT_CHAR_DISC_INF 0x07
#define ATT_CHAR_VCRD_CMD 0x08
#define ATT_CHAR_VCRD_ECM 0x0A /* Emulation Cmd */
#define ATT_CHAR_DEVI_INF 0x0D

#define S(x) #x
#define DBGCODE(n) "<" S(n) "> "

#define CLEARCHAR 10
#define	AUTHSEEDG 11
#define	AUTHTRY   12
#define	SEGSET 	  13
#define	AUTHSEEDR 14
#define	AUTHACK   15
#define	SEGACK 	  16
#define	GETVENDI  17
#define	GETVENDF  18
#define	GETDISCI  19
#define	UNCHARVR  20
#define	GETDEVII  21
#define	UNATTCDI  22
#define	VCARDEPR  23
#define	HEARTBT	  24
#define	CONNOTIF  25
#define	DISNOTIF  26
#define	BLEMOD 	  27


enum {
	SUBTYPE_HEARTB = 0x04,
	SUBTYPE_DEBUG,
	SUBTYPE_DEBUG2,
	SUBTYPE_DEBUG3,
	SUBTYPE_DEBUG4,
	SUBTYPE_DEBUG5,
	SUBTYPE_ERROR_STATUS = 0x0A,
};

enum {
	MODTYPE_TOGGLE = 0x03,
	MODTYPE_CONNECTED = 0x05,
};

#define COMMAND_DELAYS 5

static struct timeval dump_timestamp_tv;

static uint32_t is_connected = 0;
static uint32_t bug_reproduced = 0;
static uint32_t heartbeat_counter = 0;

static uint8_t last_disconnection_status = 0x00;
static uint8_t seq_num;

static void ASSERT(const char *assertion)
{
	printf("ASSERTED!!: %s", assertion);
}

static inline int64_t get_interval_with(struct timeval *time, struct timeval *time2)
{
	int64_t interval;

	interval = (time2->tv_sec - time->tv_sec) * 1000000
			  + (time2->tv_usec - time->tv_usec);

	return interval;
}

static int64_t get_interval(struct timeval *time)
{
	struct timeval now;
	gettimeofday(&now, NULL);
	return get_interval_with(time, &now);
}

uint32_t set_interface_attribs (uint32_t rxfd, uint32_t speed, uint32_t parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (rxfd, &tty) != 0)
	{
		printf ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (rxfd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (uint32_t rxfd, uint32_t should_block, uint32_t timeout)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (rxfd, &tty) != 0)
	{
		printf ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	// timeout in decisecond
	tty.c_cc[VTIME] = timeout;            // 0.5 seconds read timeout

	if (tcsetattr (rxfd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes", errno);
}

void dump_packet(uint32_t rxtx, const uint8_t *buf, uint32_t dump_type, uint8_t packet_type, 
		uint8_t packet_status1, uint8_t packet_status2)
{
	uint16_t len = (uint16_t)(buf[1] << 8) | (uint16_t)(buf[2] & 0xff);
	uint32_t i;
	uint32_t printlen = len + 4;

	switch (dump_type) {
		case DUMP_RAW:
			for (i = 0; i < printlen; i++) {
				printf("%02x", buf[i]);
				if (i != printlen - 1) printf("|");
				if (i == printlen - 1) printf("\n");
			}
			break;
		case DUMP_EXP:
			switch (packet_type) {
				case SUBTYPE_DEBUG: 
					printf("adv_stop -> STACK\n"); 
					break;
				case SUBTYPE_DEBUG2: 
					printf("gapc_cmp_evt(op:%02X|status:%02X) <- STACK\n",
							packet_status1, packet_status2);
					break;
				case SUBTYPE_DEBUG3: 
					if (packet_status1 == 0x0B) 
						printf("adv_start_nonconn -> STACK\n"); 
					else
						printf("adv_start_undir -> STACK\n"); 
					break;
				case SUBTYPE_DEBUG4: 
					if (packet_status1 == 0x0A || packet_status1 == 0x0D)  //BUG: I have no idea why 0x0A becomes nonconn
						printf("adv_comp_nonconn(0x%02X) <- STACK\n", packet_status2);
					else 
						printf("adv_comp_undir(0x%02X) <- STACK\n", packet_status2);
					break;
				default:
					ASSERT("unhandled dump packet type\n");
					break;
			}
			break;
		default:
			ASSERT("unhandled dump mode\n");
			break;
	}

	if (last_disconnection_status == 0x3E) 
		printf("[WARNING] Code: 0x3E. Do a on/off cycle of Android BLE.\n");
}


uint16_t att_cmd = 0x0000;
uint32_t packet_processor( uint32_t rxtx )
{
	uint8_t *packet = &bytebuf[0];

	struct timeval now, last;
	gettimeofday(&now, NULL);
	printf("[%010lld] [DS:%02X|CON:%3s] %s ", get_interval_with(&last, &now), 
			last_disconnection_status, is_connected ? "YES": "NO", rxtx == RXPACKET ? "a<--b" : "a-->b");
	last = now;

	uint32_t ret = 0;
	
	switch (packet[POS_TYPE]) {
		case TYPE_BLEATT:
		{		
			seq_num = packet[POS_ATT_SEQ];
			att_cmd = (packet[POS_ATT_CMD0] << 8) | packet[POS_ATT_CMD1];
			if (packet[POS_LENGTH] == 0x00 && packet[POS_LENGTH+1] == 0x02) {
				dbg(DBGCODE(CLEARCHAR)"Clearing char: %02X\n", packet[POS_ATT_CHRNUM]);
				return 0;
			}
			switch (packet[POS_ATT_CHRNUM]) {
				case ATT_CHAR_VEND_CMD:
				{
					switch (att_cmd) {
						case 0x0001: // Auth seed get
							dbg(DBGCODE(AUTHSEEDG)"auth seed get: seq(%02X)", seq_num);
							break;
						case 0x0002: // Auth try
							dbg(DBGCODE(AUTHTRY)"auth try: seq(%02X)", seq_num);
							break;
						case 0x0006: // 7SEG set
							dbg(DBGCODE(SEGSET)"7seg set: seq(%02X)", seq_num);
							break;
						default:
							ASSERT("Unrecognized ATT_CHAR_VEND_CMD code");
							dbg(" %04x", att_cmd);
							break;
					}
					break;
				}
				case ATT_CHAR_VEND_RSP:
				{
					switch (att_cmd) {
						case 0x0001: // Auth seed reply
							dbg(DBGCODE(AUTHSEEDR)"auth seed reply: seq(%02X)", seq_num);
							break;
						case 0x0002: // Auth ack
							dbg(DBGCODE(AUTHACK)"auth ack: seq(%02X)", seq_num);
							break;
						case 0x0006: // 7SEG ack
							dbg(DBGCODE(SEGACK)"7seg ack: seq(%02X)", seq_num);
							break;
						default:
							ASSERT("Unrecognized ATT_CHAR_VEND_RSP code");
							dbg(" %04x", att_cmd);
							break;
					}		
					break;
				}
				case ATT_CHAR_VEND_INF:
				{
					switch (att_cmd) {
						case 0x0042: // Get Vend info
							dbg(DBGCODE(GETVENDI)"get vend info: seq(%02X)", seq_num);
							break;
						default:
							ASSERT("Unrecognized ATT_CHAR_VEND_INF code");
							dbg(" %04x", att_cmd);
							break;
					}		
					break;
				}
				case ATT_CHAR_VEND_FNC:
				{
					switch (att_cmd) {
						case 0x0041: // Get Vend func
							dbg(DBGCODE(GETVENDF)"get vend func: seq(%02X)", seq_num);
							break;
						default:
							ASSERT("Unrecognized ATT_CHAR_VEND_FNC code");
							dbg(" %04x", att_cmd);
							break;
					}		
					break;
				}
				case ATT_CHAR_DISC_INF:
				{
					switch (att_cmd) {
						case 0x0083: // Get Disc Info
							dbg(DBGCODE(GETDISCI)"get disc info: seq(%02X)", seq_num);
							break;
						default:
							ASSERT("Unrecognized ATT_CHAR_DISC_INF code");
							dbg(" %04x", att_cmd);
							break;
					}		
					break;
				}
				case 0x09:
				{
					switch (att_cmd) {
						default:
							dbg(DBGCODE(UNCHARVR)"-- ATT_CHAR_VCRD_RSP --");
							break;
					}
					break;
				}
				
#if 0
				case ATT_CHAR_VCRD_ECM:
				{
					switch (att_cmd) {
						default:
							ASSERT("Unrecognized ATT_CHAR_VCRD_ECM code");
							dbg(" %04x", att_cmd);
							break;
					}
					break;
				}
		
#else
				case 0x0A:  /* 0x0D -> 0x0A ? (BUG) */
#endif
				case ATT_CHAR_DEVI_INF:
				{
					switch (att_cmd) {
						case 0x0f00: // Get Device Info
							dbg(DBGCODE(GETDEVII)"get device info: seq(%02X)", seq_num);
							break;
						case 0x24f7: // ???
							dbg(DBGCODE(UNATTCDI)"-- ATT_CHAR_DEVI_INF (unknown: 24F7)");
							break;
						default:
							ASSERT("Unrecognized ATT_CHAR_DEVI_INF code");
							dbg(" %04x", att_cmd);
							break;
					}		
					break;
				}
				case ATT_CHAR_VCRD_CMD:
				{
					switch (att_cmd) {
						case 0x0008: // Vcard emu pre
							dbg(DBGCODE(VCARDEPR)"vcard emu pre: seq(%02X)", seq_num);
							break;
						default:
							ASSERT("Unrecognized ATT_CHAR_VCRD_CMD code");
							dbg(" %04x", att_cmd);
							break;
					}		
					break;
				}
				default:
					ASSERT("Unrecognized ATT char code");
					dbg(" %04x", packet[POS_ATT_CHRNUM]);
					break;
			}
			heartbeat_counter = 0;
			break;
		}
		case TYPE_BLECTL:
			switch (packet[POS_SUBTYPE]) {
				case SUBTYPE_HEARTB:
					dbg(DBGCODE(HEARTBT)"Heartbeat (");
					switch (packet[POS_SUBTYPE+1]) {
						case 0x00:
							dbg("Normal");
							break;
						case 0x02:
							dbg("GAPM indicating BUSY_AIR!!");
							break;
						case 0x04:
							dbg("MEMORY_ALLOC_FAIL reset!");
							break;
						case 0x0F:
							dbg("HARDWARE_FAULT exception caught!");
							break;
						case 0x05:
							dbg("TASK_LLC stuck! (Link-Layer Abnormality) BUG!");
							break;
						case 0x06:
							dbg("ADV_STOP timed out (no ADV_COMP came)");
							break;
						default:
							dbg("Unhandled Heartbeat code: %X", packet[POS_SUBTYPE+1]);
							break;
					}
					dbg(")");
					break;
				case SUBTYPE_CONNECT:
					dbg(DBGCODE(CONNOTIF)"<<<<<< Connection notif <<<<<<<<<<<<");
					break;
				case SUBTYPE_DISCONN:
					dbg(DBGCODE(DISNOTIF)">>>>>>> Disconnection notif >>>>>>>>>>>");
					last_disconnection_status = packet[POS_SUBTYPE+1];
					is_connected = 0;
					break;
				case SUBTYPE_READY:
					dbg("Module ready!");
					break;
				case SUBTYPE_DEBUG4: // 0x08
					if (packet[POS_SUBTYPE+2] == 0x41) {
						bug_reproduced = 1;
						dbg("GAP_ERR_PROTOCOL_PROBLEM!");
					}
				case SUBTYPE_DEBUG2:
					if (packet[POS_SUBTYPE+2] == 0xEE) {
						bug_reproduced = 1;
						dbg("ADV STOP IS SENT, BUT NO RESPONSE FROM STACK!");
					}
				case SUBTYPE_DEBUG3:
				case SUBTYPE_DEBUG5:
				case SUBTYPE_DEBUG:
					if (packet_dump_type == DUMP_EXP)
						dump_packet(rxtx, packet, DUMP_EXP, packet[POS_SUBTYPE], packet[POS_SUBTYPE+1], packet[POS_SUBTYPE+2]);
					break;
				case SUBTYPE_ERROR_STATUS:
					dbg("GAPM(C)_CMP_EVT abnormal Error: %02X <---------------------------------- ", 
							packet[POS_SUBTYPE+1]);
					break;
				default:
					ASSERT("unhandled BLE control command");
					break;
			}
			break;
			case TYPE_BLEMOD:
			{
				dbg(DBGCODE(BLEMOD));
				switch(packet[POS_SUBTYPE]) {
					case MODTYPE_CONNECTED:
						dbg("BleMod: Connected mode");
						break;
					case MODTYPE_TOGGLE:
						dbg("BleMod: Toggle Mode");
						break;
					default:
						ASSERT("Unhandled BleMod Mode: ");
						dbg("%02X", packet[POS_SUBTYPE]);
						break;
				}
			}
			break;
		default:
			ASSERT("unhandled command type");
			break;
	}
	
	if (packet_dump_type == DUMP_RAW)
		//dump_packet(rxtx, packet, DUMP_RAW, 0x00, 0x00, 0x00);

	dbg("\n");

	memset(bytebuf, 0, sizeof(bytebuf));

	return ret;
}

#define NO_PACKET_YET 0
#define GOT_PACKET 1
uint32_t byte_handler(uint8_t b)
{
	bytebuf[bytepos] = b; 

	switch (bytest) {
		case BSTATE_IDLE:
			if (b == BLECOMM_PKT_HEADER) {
				bytest = BSTATE_WAIT_LEN1;
			} else {
				bytepos = 0; 
				bytetoread = 0;
				goto out;
			}
			break;
		case BSTATE_WAIT_LEN1:
			bytest = BSTATE_WAIT_LEN2;
			bytetoread = b << 8;
			break;
		case BSTATE_WAIT_LEN2:
			bytetoread |= b;
			if( bytetoread < BYTEBUFSIZE ) {
				bytest = BSTATE_WAIT_BODY;
			}
			else {
				bytepos = 0;
				bytetoread = 0;
				bytest = BSTATE_IDLE;
				goto out;
			}
			break;
		case BSTATE_WAIT_BODY:
			/* if this is byte after the last, then it must be a stop bit */
			if (bytetoread == 0) {
				bytepos = 0; 
				bytest = BSTATE_IDLE;
				if (b == BLECOMM_PKT_STOP) {
					return GOT_PACKET;
				}
			} else {
				bytetoread--;
			}
			break;
		default:
			break;
	}

	if (bytepos < BYTEBUFSIZE) {
		bytepos++;
	}

out:
	return NO_PACKET_YET;
}

static volatile uint32_t keep_running = 1;
void signal_handler(uint32_t d)
{
	keep_running = 0;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		printf("usage %s <rx serial device 1> [tx serial device 2]\n", argv[0]);
		return -1;
	}

	rxfd = open(argv[1], O_RDONLY | O_NOCTTY | O_SYNC); 
	if (rxfd < 0) {
		printf("error %d opening %s (%s)\n", errno, argv[1], strerror(errno));
	}

	if (argc == 3) {
		txfd = open(argv[2], O_RDONLY | O_NOCTTY | O_SYNC);
		if (txfd < 0) {
			printf("error %d opening %s (%s)\n", errno, argv[2], strerror(errno));
		}
	} else {
		txfd = 0;
	}


#define SHOULD_BLOCK 1
#define DONT_BLOCK 0
	set_interface_attribs(rxfd, B57600, 0);
	set_blocking(rxfd, DONT_BLOCK, 0);

	if (txfd != 0) {
		set_interface_attribs(txfd, B57600, 0);
		set_blocking(txfd, DONT_BLOCK, 0);
	}

	set_blocking(STDIN_FILENO, DONT_BLOCK, 1);

	// write something  
	uint8_t rxbuf = 0, txbuf = 0;
	uint32_t rxres = 0, txres = 0;

	rxbuf = 0;

	signal(SIGINT, (__sighandler_t)signal_handler);

	printf("RX: %s, TX: %s\n", (char *)argv[1], txfd != 0 ? (char *)argv[2] : "NULL");

	// initial timestamp now (for dumping packets)
	gettimeofday(&dump_timestamp_tv, NULL);
	while (keep_running) {
		rxres = read(rxfd, &rxbuf, 1);
		if (txfd != 0) txres = read(txfd, &txbuf, 1);

		if (rxres > 0) 
			if (byte_handler(rxbuf))
				packet_processor(RXPACKET);

		if (txres > 0 && txfd != 0)
			if (byte_handler(txbuf))
				packet_processor(TXPACKET);

		rxbuf = 0; txbuf = 0;
	}

	close(rxfd);
	if (txfd != 0) close(txfd);

	return 0;
}
