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
#define UPDATE_APP

#ifdef DEBUG_PRINT
#define dbg(...) printf(__VA_ARGS__)
#else
#define dbg(...)
#endif

#define SET_DELAYED(pkt, delay) \
	if (delayed_write_done) { \
		memcpy(writebuf, pkt, sizeof(pkt)); \
		set_delayed_write(delay, sizeof(pkt)); \
	}
#define SET_DELAYED_NEXT(pkt, delay, next) \
	if (delayed_write_done) { \
		memcpy(writebuf, pkt, sizeof(pkt)); \
		set_delayed_write(delay, sizeof(pkt)); \
		machst = next; \
	} 

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
#ifdef UPDATE_APP
  MSTATE_AUTH2_NEGO,
  MSTATE_NEGO_OK,
  MSTATE_UPDATE_REQ,
  MSTATE_UPDATE_FIN,
  MSTATE_UPDATE_DAT,
  MSTATE_INIT_VFUNC,
  MSTATE_INIT_VINFO,
#endif
  MSTATE_AFTER_AUTH,
} mstate;

mstate machst = MSTATE_IDLE;
mstate prev_machst = MSTATE_IDLE;

enum {
  POS_HEAD = 0,
  POS_LENGTH,
  POS_TYPE = 3,
  POS_SUBTYPE = 4,
  POS_PAYLOAD = 4
};

#define TYPE_BLEATT 0xB2 // gatt related
#define TYPE_BLEMOD 0xB3 // command from here to ble
#define TYPE_BLECTL 0xB4 // command from ble to here

#define SUBTYPE_READY   0x02
#define SUBTYPE_DISCONN 0x01
#define SUBTYPE_CONNECT 0x00


enum {
  SUBTYPE_HEARTB = 0x04,
  SUBTYPE_DEBUG,
  SUBTYPE_DEBUG2,
  SUBTYPE_DEBUG3,
  SUBTYPE_DEBUG4,
  SUBTYPE_DEBUG5,
  SUBTYPE_ERROR_STATUS = 0x0A,
};

#define COMMAND_DELAYS 1

static struct timeval delayed_write_tv;
static struct timeval dump_timestamp_tv;
static struct timeval connection_timeout_tv;;
static struct timeval heartb_time_tv;

#ifdef UPDATE_APP
static int offset_addr = 0;
#endif
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
#ifndef UPDATE_APP
static unsigned char pkt_auth_ok[] = { 0x3A, 0x00, 0x1A, 0xB2, 0x06, 
                                       0x00, 0x16,  /* att pkt size */
                                       0x00, /* seq */
                                       0x00, 0x01, /* type */
                                       0x01, 0x00, /* body size (16) */
                                       /* auth seed */
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF,
                                       0x0A };
#else
static unsigned char pkt_auth_ok[] = { 0x3A, 0x00, 0x1A, 0xB2, 0x06, 
                                       0x00, 0x16,  /* att pkt size */
                                       0x00, /* seq */
                                       0x7E, 0x01, /* type */
                                       0x00, 0x11, /* body size (16) */
                                       /* auth seed */
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 0xFF, 0xFF, 
                                       0xFF, 
								       0x6D, /* err code */
                                       0x0A };

static unsigned char pkt_auth2_ok[] = { 0x3A, 0x00, 0x0A, 0xB2, 0x06,
									    0x00, 0x06,
										0x00,
										0x7E, 0x02,
										0x00, 0x01,
										0x6D,
										0x0A };

static unsigned char pkt_init_vendfunc[] = { 0x3A, 0x00, 0x0B, 0xB2, 0x00,
											 0x00, 0x07,
											 0x00,
											 0x00, 0x41,
											 0x00, 0x02,
											 0x03, 0x00,
											 0x0A
											};
static unsigned char pkt_init_vendinfo[] = { 0x3A, 0x00, 0x18, 0xB2, 0x01,
											 0x00, 0x14,
											 0x00,
											 0x00, 0x42,
											 0x00, 0x0F,
											 0x00, 0x00, 0x00, 0x00, 0x00, /* machine code */
											 0x00, /* bottler code */
											 0x00, 0x00, 0x00, /* area code */
											 0x00, /* money brand code (1 only) */
											 /* 0x01, 0x01, // money 1 brand number */
											 0x00, 0x00, 0x00, 0x00, 0x00,
											 0x0A };
static unsigned char pkt_update_rsp[] = { 0x3A, 0x00, 0x0B, 0xB2, 0x06,
										  0x00, 0x07,
										  0x7E, 0xE1,
										  0x00, 
										  0x00, 0x00, 0x00, 0x00,
										  0x0A };
static unsigned char pkt_upd_dat_rsp[] = { 0x3A, 0x00, 0x07, 0xB2, 0x0A,
										  0x00, 0x03,
										  0x7E, 0xE2,
										  0x00, 
										  0x0A };
static unsigned char pkt_upd_fin_rsp[] = { 0x3A, 0x00, 0x07, 0xB2, 0x0A,
										  0x00, 0x03,
										  0x7E, 0xE3,
										  0x00, 
										  0x0A };
#endif
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

static int multi_times = 0;
static int multi_ms = 0;
static int multi_packet_len = 0;

static void set_delayed_write_multi(int times, int ms, int packet_len)
{
  if (multi_times == 0) {
    multi_times = times;
    multi_ms = ms;
    multi_packet_len = packet_len;
    set_delayed_write(multi_ms, multi_packet_len);
  } else {
    dbg("not finished previous multi-write\n");
  }
}

static void check_delayed_write_multi(void)
{
  if (multi_times > 0) {
    multi_times--;
    set_delayed_write(multi_ms, multi_packet_len);
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
  struct timeval now;

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


int packet_processor( void )
{
  unsigned char *packet = &bytebuf[0];

#if 0
  if (packet_dump_type == DUMP_RAW)
    dump_packet(packet, DUMP_RAW, 0x00, 0x00, 0x00);
#endif

  int ret = 0;

  switch (packet[POS_TYPE]) {
    case TYPE_BLEATT:
      if (packet[POS_SUBTYPE] == 0x05) {
        seq_num = packet[POS_SUBTYPE+3];
        if (packet[POS_SUBTYPE+5] == 0x06) { // 7SEG set
          dbg("7seg setting: seq(%02X)\n", seq_num);
          machst = MSTATE_LED_NEGO;
        } else if (packet[POS_SUBTYPE+5] == 0x01) { // Auth
          dbg("auth init: seq(%02X)\n", seq_num);
          machst = MSTATE_AUTH_NEGO;
#ifdef UPDATE_APP
		} else if (packet[POS_SUBTYPE+5] == 0x02) { // Auth2 req
		  dbg("auth2 init: seq(%02X)\n", seq_num);
		  machst = MSTATE_AUTH2_NEGO;
#endif
        } else {
          machst = MSTATE_IDLE;
        }
	  }
#ifdef UPDATE_APP
	  else if (packet[POS_SUBTYPE] == 0x0B) { 
		if (packet[POS_SUBTYPE+4] == 0xE1) { // update req
		  dbg("update req: seq(%02X)\n", seq_num);
		  machst = MSTATE_UPDATE_REQ;
		} else if (packet[POS_SUBTYPE+4] == 0xE2) {
		  int prev_offset_addr = offset_addr;
		  offset_addr = *(int *)&packet[POS_SUBTYPE+5];
		  offset_addr = ntohl(offset_addr) >> 8;
		  if (!(prev_offset_addr + 1 == offset_addr || prev_offset_addr == offset_addr || offset_addr == 0))
			dbg("missed packet!\n");
		  //dbg("update data %08x\n", offset_addr);
		  machst = MSTATE_UPDATE_DAT;
		} else if (packet[POS_SUBTYPE+4] == 0xE3) {
		  dbg("update finish\n");
		  machst = MSTATE_UPDATE_FIN;
		}
	  } else {
		dbg("unknown char code: %02X\n", packet[POS_SUBTYPE]);
	  }
#endif
      heartbeat_counter = 0;
      break;
    case TYPE_BLECTL:
      switch (packet[POS_SUBTYPE]) {
        case SUBTYPE_HEARTB:
		  dbg("%05d: Heartbeat(%d)\n", get_interval(&heartb_time_tv)/1000, packet[POS_SUBTYPE+1]);
		  gettimeofday(&heartb_time_tv, NULL);
          if (is_connected) {
            heartbeat_counter++;
            if (heartbeat_counter > 2)
              machst = MSTATE_IDLE; // force disconnection 
          } else {
            machst = MSTATE_IDLE; // if not connected, just set to toggle every heartbeat
          }
          if (bug_reproduced) {
            printf("Bug reproduced, trying to set to TOGGLE\n");
            machst = MSTATE_IDLE;
          }
          switch (packet[POS_SUBTYPE+1]) {
            case 0x00:
              // normal
              break;
            case 0x02:
              printf("GAPM indicating BUSY_AIR!!\n");
              break;
            case 0x04:
              printf("MEMORY_ALLOC_FAIL reset!\n");
              break;
            case 0x0F:
              printf("HARDWARE_FAULT exception caught!\n");
              break;
            case 0x05:
              printf("TASK_LLC stuck! (Link-Layer Abnormality) BUG!\n");
              break;
            case 0x06:
              printf("ADV_STOP timed out (no ADV_COMP came)\n");
              break;
            default:
              dbg("Unhandled Heartbeat code: %X\n", packet[POS_SUBTYPE+1]);
              break;
          }
          break;
        case SUBTYPE_CONNECT:
		  dbg("Connected!\n");
          machst = MSTATE_CONNECTED;
          break;
        case SUBTYPE_DISCONN:
          dbg("Disconnected (%02X) : Disconnect delay: %d ms!\n", packet[POS_SUBTYPE+1] & 0xff, 
              delayed_disconnection);
          last_disconnection_status = packet[POS_SUBTYPE+1];
          is_connected = 0;
		  prev_machst = machst;
          machst = MSTATE_DISCONNECTED;
          break;
        case SUBTYPE_READY:
          dbg("Module ready!\n");
          machst = MSTATE_READY;
          break;
        case SUBTYPE_DEBUG4: // 0x08
          if (packet[POS_SUBTYPE+2] == 0x41) {
            bug_reproduced = 1;
            printf("GAP_ERR_PROTOCOL_PROBLEM!\n");
          }
        case SUBTYPE_DEBUG2:
          if (packet[POS_SUBTYPE+2] == 0xEE) {
            bug_reproduced = 1;
            printf("ADV STOP IS SENT, BUT NO RESPONSE FROM STACK!\n");
          }
        case SUBTYPE_DEBUG3:
        case SUBTYPE_DEBUG5:
        case SUBTYPE_DEBUG:
          if (packet_dump_type == DUMP_EXP)
            dump_packet(packet, DUMP_EXP, packet[POS_SUBTYPE], packet[POS_SUBTYPE+1], packet[POS_SUBTYPE+2]);
          break;
        case SUBTYPE_ERROR_STATUS:
          printf("GAPM(C)_CMP_EVT abnormal Error: %02X <---------------------------------- \n", 
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

  memset(bytebuf, 0, sizeof(bytebuf));

  return ret;
}

int ddelay = 400;
int vendfunc_done = 0;

int state_machine() {
  int ret = 0;
  int rseed = time(NULL);

  ant = ant == 0x32 ? 0x10 : 0x32;
  switch (machst) {
    case MSTATE_READY: 
#ifdef UPDATE_APP
      	/* init ble data */
	  if (delayed_write_done) {
		  memcpy(writebuf, pkt_init_vendinfo, sizeof(pkt_init_vendinfo));
		  set_delayed_write(COMMAND_DELAYS, sizeof(pkt_init_vendinfo));
		  machst = MSTATE_INIT_VFUNC;
	  }
	  break;
	case MSTATE_INIT_VFUNC:
	  SET_DELAYED_NEXT(pkt_init_vendfunc, COMMAND_DELAYS, MSTATE_INIT_VINFO);
	  break;
	case MSTATE_INIT_VINFO:
	  SET_DELAYED_NEXT(pkt_toggle, COMMAND_DELAYS, MSTATE_TOGGLE);
	  break;
#endif
    case MSTATE_DISCONNECTED:
#ifdef UPDATE_APP
	  SET_DELAYED_NEXT(pkt_toggle, COMMAND_DELAYS, prev_machst);
	  break;
#endif
    case MSTATE_IDLE:
#ifdef UPDATE_APP
#else
      // change to toggle mode
      dbg("Switching to toggle..\n");
      // change packet toggle antenna
      pkt_toggle[5] = ant;
	  SET_DELAYED_NEXT(pkt_toggle, COMMAND_DELAYS, MSTATE_TOGGLE); 
#endif
      break;
    case MSTATE_TOGGLE:
      //dbg("Waiting for connection..\n");
      // wait for connection (do nothing)
      break;
    case MSTATE_CONNECTED:
#ifndef UPDATE_APP
      // change to connected mode
	  SET_DELAYED_NEXT(pkt_connected, COMMAND_DELAYS, MSTATE_IN_CONNECTION);
      set_connection_timeout(3*1000);
#endif
      is_connected = 1;
      last_disconnection_status = 0x00;
      break;
    case MSTATE_IN_CONNECTION:
      // if no negotiation after a long time, force disconnect
      if (get_interval(&connection_timeout_tv) > connection_timeout_us) {
        machst = MSTATE_IDLE;
      }
      break;
    case MSTATE_LED_NEGO:
      pkt_led_ok[7] = seq_num;
	  SET_DELAYED_NEXT(pkt_led_ok, COMMAND_DELAYS, MSTATE_IN_CONNECTION);
      break;
    case MSTATE_AUTH_NEGO:
      pkt_auth_ok[7] = seq_num;
	  SET_DELAYED_NEXT(pkt_auth_ok, COMMAND_DELAYS, MSTATE_IN_CONNECTION);
      break;
#ifdef UPDATE_APP
	case MSTATE_AUTH2_NEGO:
	  // just send ok
      pkt_auth2_ok[7] = seq_num;
	  SET_DELAYED_NEXT(pkt_auth2_ok, COMMAND_DELAYS, MSTATE_IN_CONNECTION);
	  break;
	case MSTATE_UPDATE_REQ:
	  // just send ok
	  SET_DELAYED_NEXT(pkt_update_rsp, COMMAND_DELAYS, MSTATE_IN_CONNECTION);
	  break;
	case MSTATE_UPDATE_DAT:
	  // just send ok
	  SET_DELAYED_NEXT(pkt_upd_dat_rsp, COMMAND_DELAYS, MSTATE_IN_CONNECTION);
	  break;
	case MSTATE_UPDATE_FIN:
	  // just send ok
	  SET_DELAYED_NEXT(pkt_upd_fin_rsp, COMMAND_DELAYS, MSTATE_IN_CONNECTION);
	  break;
#endif
    case MSTATE_AFTER_AUTH:
	  SET_DELAYED_NEXT(pkt_force_disconnect, delayed_disconnection, MSTATE_FORCE_DISCONNECT);
      if (delayed_write_done) {
        // wait a bit and then force disconnection
        //delayed_disconnection = rand_r(&rseed) % 200 + 1100; // randomly between 1100 ms to 1300 ms
        delayed_disconnection = rand_r(&rseed) % 35 + ddelay; // randomly between 45 ms to 49 ms
        dbg("Auth response sent. Disconnecting in %d ms..\n", delayed_disconnection);
      }
      break;
    case MSTATE_FORCE_DISCONNECT:
      // wait until delayed force disconnect packet is sent
      if (delayed_write_done)
        machst = MSTATE_IDLE;
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
  gettimeofday(&heartb_time_tv, NULL);
  while (keep_running) {
    res = read(fd, &buf, 1);

    if (res > 0)
      if(byte_handler(buf))
         packet_processor();

    state_machine();

    if (delayed_write()) {
//      dbg("Wrote delayed write\n");
#if 0
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

      check_delayed_write_multi();
    }

	// check last heartb
	if (get_interval(&heartb_time_tv) > 20000000) { // more than 20 seconds 
		dbg("HEARTBEAT TIMEOUT!\n");
		gettimeofday(&heartb_time_tv, NULL);
	}
      
    buf = 0;
    wlen = 0;
  }

  close(fd);
}
