/* Copyright @ Isa Ansharullah of aLab inc. 2016 */

#include <stdio.h>
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

int fd; // file descriptor for our serial comm

#define DUMP_RAW 1 // dump the raw packet
#define DUMP_EXP 2 // dump the packet with explanation

static int packet_dump_type = DUMP_RAW;

#define BYTEBUFSIZE 512
#define WRITEBUFSIZE 512

unsigned char bytebuf[BYTEBUFSIZE];
unsigned short bytepos;
unsigned short bytetoread;

unsigned char writebuf[WRITEBUFSIZE];

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

typedef enum machine_state {
  MSTATE_IDLE,
  MSTATE_READY,
  MSTATE_TOGGLE,
  MSTATE_CONNECTED,
  MSTATE_FORCE_DISCONNECT,
  MSTATE_DISCONNECTED,
  MSTATE_IN_CONNECTION,
  MSTATE_LED_NEGO,
  MSTATE_AUTH_NEGO,
  MSTATE_AUTH_TRIAL,
} mstate;

mstate machst = MSTATE_IDLE;

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
#define ATT_CHAR_DEVI_INF 0x0D

enum {
  SUBTYPE_HEARTB = 0x04,
  SUBTYPE_DEBUG,
  SUBTYPE_DEBUG2,
  SUBTYPE_DEBUG3,
  SUBTYPE_DEBUG4,
  SUBTYPE_DEBUG5,
  SUBTYPE_ERROR_STATUS = 0x0A,
};

#define COMMAND_DELAYS 5

static struct timeval delayed_write_tv;
static struct timeval dump_timestamp_tv;
static struct timeval connection_timeout_tv;;

static long delayed_write_us = LONG_MAX;
static long connection_timeout_us = LONG_MAX;
static int delayed_write_done = 1;
static int delayed_write_len = 0;
static int delayed_disconnection = 1100; // initial value of delayed disconnection (ms)
static int is_connected = 0;
static int bug_reproduced = 0;
static int heartbeat_counter = 0;

static unsigned char last_disconnection_status = 0x00;

static unsigned char seq_num;
static unsigned char pkt_toggle[] = { 0x3A, 0x00, 0x06, 0xB3, 0x03, 0x30, 0x00, 0x00, 0x00, 0x0A };
static unsigned char pkt_connected[] = { 0x3A, 0x00, 0x06, 0xB3, 0x05, 0x33, 0x00, 0x00, 0x00, 0x0A };
/* ATT PACKETS */
static unsigned char pkt_led_ok[] = { 0x3A, 0x00, 0x0A, 0xB2, 0x06, 
                                      0x00, 0x06, /* att pkt size */
                                      0x00, /* seq */
                                      0x00, 0x06, /* type */
                                      0x00, 0x01, /* body size */
                                      0x00, /* led success */
                                      0x0A };
static unsigned char pkt_auth_ok[] = { 0x3A, 0x00, 0x1A, 0xB2, 0x06, 
                                       0x00, 0x16,  /* att pkt size */
                                       0x00, /* seq */
                                       0x00, 0x01, /* type */
                                       0x00, 0x11, /* body size (16) */
									   0x00, /* success */
                                       /* auth seed */
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF,
                                       0x0A };
static unsigned char pkt_force_disconnect[] = { 0x3A, 0x00, 0x0A, 0xB2, 0x08,
                                                0x00, 0x06, /* att pkt size */
                                                0x00, /* seq */
                                                0x00, 0x07, /* type */
                                                0x00, 0x01, /* body size */
                                                0x03, /* disconnect request */
                                                0x0A };


static unsigned char ant = 0x32;

static void ASSERT(const char *assertion)
{
  printf("ASSERTED!!!!!!!!!!!!!!!: %s\n", assertion);
  //while(1);
}

static long get_interval(struct timeval *time)
{
  struct timeval now;
  long interval;

  gettimeofday(&now, NULL);
  interval = (now.tv_sec - time->tv_sec) * 1000000
    + (now.tv_usec - time->tv_usec);

  return interval;
}

static void set_connection_timeout(int ms)
{
  connection_timeout_us = ms * 1000;
  gettimeofday(&connection_timeout_tv, NULL); 
}

static void set_delayed_write(int ms, int packet_len)
{
  if (!delayed_write_done) {
    dbg("Should not write!, last write isn't done yet!\n");
  } else {
    gettimeofday(&delayed_write_tv, NULL);
    delayed_write_us = ms * 1000;
    delayed_write_done = 0;
    delayed_write_len = packet_len;
    //dbg("setting delayed write next: %ld\n", delayed_write_us);
  }
}

static int delayed_write(void)
{
  struct timeval now;
  long interval;

  gettimeofday(&now, NULL);
  interval = (now.tv_sec - delayed_write_tv.tv_sec) * 1000000
    + (now.tv_usec - delayed_write_tv.tv_usec);

  //dbg("interval: %ld, us: %ld\n", interval, delayed_write_us);

  return (interval < delayed_write_us) ? 0 : 1;
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
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

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block, int timeout)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        // timeout in decisecond
        tty.c_cc[VTIME] = timeout;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}

void dump_packet(const unsigned char *buf, int dump_type, unsigned char packet_type, 
    unsigned char packet_status1, unsigned char packet_status2)
{
  unsigned short len = (unsigned short)(buf[1] << 8) | (unsigned short)(buf[2] & 0xff);
  int i;
  int printlen = len + 4;

  printf("[%010ld] [DS:%02X|CON:%3s] ", get_interval(&dump_timestamp_tv)/1000, 
      last_disconnection_status, is_connected ? "YES": "NO");

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
          ASSERT("unhandled dump packet type");
          break;
      }
      break;
    default:
      ASSERT("unhandled dump mode");
      break;
  }

  if (last_disconnection_status == 0x3E) 
    printf("[WARNING] Code: 0x3E. Do a on/off cycle of Android BLE.\n");
}

unsigned short att_cmd = 0x0000;
int packet_processor( void )
{
  unsigned char *packet = &bytebuf[0];

  if (packet_dump_type == DUMP_RAW)
    dump_packet(packet, DUMP_RAW, 0x00, 0x00, 0x00);

  int ret = 0;

  switch (packet[POS_TYPE]) {
#if 0
    case TYPE_BLEATT:
      if (packet[POS_SUBTYPE] == 0x05) {
        seq_num = packet[POS_SUBTYPE+3];
        if (packet[POS_SUBTYPE+5] == 0x06) { // 7SEG set
          dbg("7seg setting: seq(%02X)\n", seq_num);
          machst = MSTATE_LED_NEGO;
        } else if (packet[POS_SUBTYPE+5] == 0x01) { // Auth
          dbg("auth init: seq(%02X)\n", seq_num);
          machst = MSTATE_AUTH_NEGO;
        } else {
          machst = MSTATE_IDLE;
        }
      }
      heartbeat_counter = 0;
      break;
#else
	case TYPE_BLEATT:
	{		
		seq_num = packet[POS_ATT_SEQ];
		att_cmd = (packet[POS_ATT_CMD0] << 8) | packet[POS_ATT_CMD1];
		machst = MSTATE_IDLE;
		switch (packet[POS_ATT_CHRNUM]) {
			case ATT_CHAR_VEND_CMD:
			{
				switch (att_cmd) {
					case 0x0001: // Auth seed get
						dbg("auth seed get: seq(%02X)", seq_num);
						machst = MSTATE_AUTH_NEGO;
						break;
					case 0x0002: // Auth try
						dbg("auth try: seq(%02X)", seq_num);
						machst = MSTATE_AUTH_TRIAL;
						break;
					case 0x0006: // 7SEG set
						dbg("7seg set: seq(%02X)", seq_num);
						machst = MSTATE_LED_NEGO;
						break;
					default:
						ASSERT("Unrecognized ATT_CHAR_VEND_CMD code");
						dbg(" %d", att_cmd);
						break;
				}
				break;
			}
			case ATT_CHAR_VEND_RSP:
			{
				switch (att_cmd) {
					case 0x0001: // Auth seed reply
						dbg("auth seed reply: seq(%02X)", seq_num);
						break;
					case 0x0002: // Auth ack
						dbg("auth ack: seq(%02X)", seq_num);
						break;
					case 0x0006: // 7SEG ack
						dbg("7seg ack: seq(%02X)", seq_num);
						break;
					default:
						ASSERT("Unrecognized ATT_CHAR_VEND_RSP code");
						dbg(" %d", att_cmd);
						break;
				}		
				break;
			}
			case ATT_CHAR_VEND_INF:
			{
				switch (att_cmd) {
					case 0x0042: // Get Vend info
						dbg("get vend info: seq(%02X)", seq_num);
						break;
					default:
						ASSERT("Unrecognized ATT_CHAR_VEND_INF code");
						dbg(" %d", att_cmd);
						break;
				}		
				break;
			}
			case ATT_CHAR_VEND_FNC:
			{
				switch (att_cmd) {
					case 0x0041: // Get Vend func
						dbg("get vend func: seq(%02X)", seq_num);
						break;
					default:
						ASSERT("Unrecognized ATT_CHAR_VEND_FNC code");
						dbg(" %d", att_cmd);
						break;
				}		
				break;
			}
			case ATT_CHAR_DISC_INF:
			{
				switch (att_cmd) {
					case 0x0083: // Get Disc Info
						dbg("get disc info: seq(%02X)", seq_num);
						break;
					default:
						ASSERT("Unrecognized ATT_CHAR_DISC_INF code");
						dbg(" %d", att_cmd);
						break;
				}		
				break;
			}
			case 0x0A: /* 0x0D -> 0x0A serial-usb bug? */
			case ATT_CHAR_DEVI_INF:
			{
				switch (att_cmd) {
					case 0x0f00: // Get Device Info
						dbg("get device info: seq(%02X)", seq_num);
						break;
					default:
						ASSERT("Unrecognized ATT_CHAR_DEVI_INF code");
						dbg(" %d", att_cmd);
						break;
				}		
				break;
			}
			case ATT_CHAR_VCRD_CMD:
			{
				switch (att_cmd) {
					case 0x0008: // Vcard emu pre
						dbg("vcard emu pre: seq(%02X)", seq_num);
						break;
					default:
						ASSERT("Unrecognized ATT_CHAR_VCRD_CMD code");
						dbg(" %d", att_cmd);
						break;
				}		
				break;
			}
			default:
				ASSERT("Unrecognized ATT char code");
				dbg(" %d", packet[POS_ATT_CHRNUM]);
				break;
		}
		heartbeat_counter = 0;
		break;
	}
#endif
    case TYPE_BLECTL:
      switch (packet[POS_SUBTYPE]) {
        case SUBTYPE_HEARTB:
          if (is_connected) {
            heartbeat_counter++;
            if (heartbeat_counter > 2)
              machst = MSTATE_IDLE; // force disconnection 
          } else {
            machst = MSTATE_IDLE; // if not connected, just set to toggle every heartbeat
          }
          if (bug_reproduced) {
            dbg("Bug reproduced, trying to set to TOGGLE");
            machst = MSTATE_IDLE;
          }
          switch (packet[POS_SUBTYPE+1]) {
            case 0x00:
              // normal
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
              dbg("Unhandled Heartbeat code: %X\n", packet[POS_SUBTYPE+1]);
              break;
          }
          break;
        case SUBTYPE_CONNECT:
          machst = MSTATE_CONNECTED;
          break;
        case SUBTYPE_DISCONN:
          dbg("Disconnected (%02X) : Disconnect delay: %d ms!\n", packet[POS_SUBTYPE+1] & 0xff, 
              delayed_disconnection);
          last_disconnection_status = packet[POS_SUBTYPE+1];
          is_connected = 0;
          machst = MSTATE_DISCONNECTED;
          break;
        case SUBTYPE_READY:
          dbg("Module ready!\n");
          machst = MSTATE_READY;
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
            dump_packet(packet, DUMP_EXP, packet[POS_SUBTYPE], packet[POS_SUBTYPE+1], packet[POS_SUBTYPE+2]);
          break;
        case SUBTYPE_ERROR_STATUS:
          dbg("GAPM(C)_CMP_EVT abnormal Error: %02X <-", 
              packet[POS_SUBTYPE+1]);
          break;
        default:
          ASSERT("unhandled BLE control command");
          break;
      }
      break;
    default:
      ASSERT("unhandled command type");
      break;
  }
  dbg("\n");

  memset(bytebuf, 0, sizeof(bytebuf));

  return ret;
}

int ddelay = 400;

int state_machine() {
  int ret = 0;
  int rseed = time(NULL);

  ant = ant == 0x32 ? 0x10 : 0x32;
  switch (machst) {
    case MSTATE_DISCONNECTED:
    case MSTATE_IDLE:
    case MSTATE_READY:
      // change to toggle mode
      dbg("Switching to toggle..\n");
      // change packet toggle antenna
      pkt_toggle[5] = ant;
      memcpy(writebuf, pkt_toggle, sizeof(pkt_toggle));
      set_delayed_write(COMMAND_DELAYS, sizeof(pkt_toggle));
      machst = MSTATE_TOGGLE;
      break;
    case MSTATE_TOGGLE:
      //dbg("Waiting for connection..\n");
      // wait for connection (do nothing)
      break;
    case MSTATE_CONNECTED:
      dbg("Connected!\n");
      // change to connected mode
      memcpy(writebuf, pkt_connected, sizeof(pkt_connected));
      set_delayed_write(COMMAND_DELAYS, sizeof(pkt_connected));
      set_connection_timeout(3*1000);
      is_connected = 1;
      last_disconnection_status = 0x00;
      machst = MSTATE_IN_CONNECTION;
      break;
    case MSTATE_IN_CONNECTION:
      // if no negotiation after a long time, force disconnect
      if (get_interval(&connection_timeout_tv) > connection_timeout_us) {
        machst = MSTATE_IDLE;
      }
      break;
    case MSTATE_LED_NEGO:
      pkt_led_ok[7] = seq_num;
      memcpy(writebuf, pkt_led_ok, sizeof(pkt_led_ok));
      set_delayed_write(COMMAND_DELAYS, sizeof(pkt_led_ok));
      machst = MSTATE_IN_CONNECTION;
      break;
    case MSTATE_AUTH_NEGO:
      pkt_auth_ok[7] = seq_num;
      memcpy(writebuf, pkt_auth_ok, sizeof(pkt_auth_ok));
      set_delayed_write(COMMAND_DELAYS, sizeof(pkt_auth_ok));
      machst = MSTATE_IN_CONNECTION;
      break;
    case MSTATE_AUTH_TRIAL:
      if (delayed_write_done) {
        // wait a bit and then force disconnection
        memcpy(writebuf, pkt_force_disconnect, sizeof(pkt_force_disconnect));
        set_delayed_write(delayed_disconnection, sizeof(pkt_force_disconnect));
        //delayed_disconnection = rand_r(&rseed) % 200 + 1100; // randomly between 1100 ms to 1300 ms
        delayed_disconnection = rand_r(&rseed) % 200 + ddelay; // randomly between 45 ms to 49 ms
        machst = MSTATE_FORCE_DISCONNECT;
      }
	  break;
#if 0
	  if (delayed_write_done) {
        memcpy(writebuf, pkt_force_disconnect, sizeof(pkt_force_disconnect));
        set_delayed_write(delayed_disconnection, sizeof(pkt_force_disconnect));
	  }
	  break;
	case MSTATE_AUTH_
      if (delayed_write_done) {
        // wait a bit and then force disconnection
        memcpy(writebuf, pkt_force_disconnect, sizeof(pkt_force_disconnect));
        set_delayed_write(delayed_disconnection, sizeof(pkt_force_disconnect));
        //delayed_disconnection = rand_r(&rseed) % 200 + 1100; // randomly between 1100 ms to 1300 ms
        delayed_disconnection = rand_r(&rseed) % 200 + ddelay; // randomly between 45 ms to 49 ms
        machst = MSTATE_FORCE_DISCONNECT;
      }
      break;
#endif
    case MSTATE_FORCE_DISCONNECT:
      // wait until delayed force disconnect packet is sent
      if (delayed_write_done) {
        machst = MSTATE_IDLE;
      }
      break;
    default:
      ASSERT("unhandled state machine state");
      break;
  }

  return ret;
}


#define NO_PACKET_YET 0
#define GOT_PACKET 1
int byte_handler(unsigned char b)
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
          //dump_packet(&bytebuf[0]);
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

static volatile int keep_running = 1;
void signal_handler(int d)
{
  keep_running = 0;
}

int main(int argc, char **argv)
{
  if (argc < 2) {
    printf("usage %s <serial device>\n", argv[0]);
    return -1;
  }

  fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC); 
  if (fd < 0) {
    printf("error %d opening %s (%s)\n", errno, argv[1], strerror(errno));
  }

#define SHOULD_BLOCK 1
#define DONT_BLOCK 0
  set_interface_attribs(fd, B57600, 0);
  set_blocking(fd, DONT_BLOCK, 0);
  set_blocking(STDIN_FILENO, DONT_BLOCK, 1);
  
  // write something  
  char buf;
  int res, i = 0;
  int cnt = 0;
  int wlen = 0;

  buf = 0;

  signal(SIGINT, signal_handler);

  // initial timestamp now (for dumping packets)
  gettimeofday(&dump_timestamp_tv, NULL);
  while (keep_running) {
    res = read(fd, &buf, 1);

    if (res > 0)
      if(byte_handler(buf))
         packet_processor();

    state_machine();

    if (delayed_write()) {
#if 1
	  printf("---> ");
      for(i = 0; i < delayed_write_len; i++) {
		printf("%02X ", writebuf[i]);
	    if (i == delayed_write_len - 1) printf("\n");
	  }
#endif

      write(fd, writebuf, delayed_write_len);
      delayed_write_us = LONG_MAX;  
      delayed_write_done = 1;
      delayed_write_len = 0;
      memset(&delayed_write_tv, 0, sizeof(delayed_write_tv));
    }

    buf = 0;
    wlen = 0;
  }

  close(fd);

  return 0;
}
