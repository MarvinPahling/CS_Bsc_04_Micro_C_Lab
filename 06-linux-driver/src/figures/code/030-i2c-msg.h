#include <linux/types.h>

/**
 * struct i2c_msg - an I2C transaction segment beginning with START
 *
 * @addr: Slave address, either 7 or 10 bits. When this is a 10 bit address,
 *   %I2C_M_TEN must be set in @flags and the adapter must support
 *   %I2C_FUNC_10BIT_ADDR.
 *
 * @flags:
 *   Supported by all adapters:
 *   %I2C_M_RD: read data (from slave to master). Guaranteed to be 0x0001! If
 *   not set, the transaction is interpreted as write.
 *
 *   Optional:
 *   %I2C_M_DMA_SAFE: the buffer of this message is DMA safe. Makes only sense
 *     in kernelspace, because userspace buffers are copied anyway
 *
 *   Only if I2C_FUNC_10BIT_ADDR is set:
 *   %I2C_M_TEN: this is a 10 bit chip address
 *
 *   Only if I2C_FUNC_SMBUS_READ_BLOCK_DATA is set:
 *   %I2C_M_RECV_LEN: message length will be first received byte
 *
 *   Only if I2C_FUNC_NOSTART is set:
 *   %I2C_M_NOSTART: skip repeated start sequence

 *   Only if I2C_FUNC_PROTOCOL_MANGLING is set:
 *   %I2C_M_NO_RD_ACK: in a read message, master ACK/NACK bit is skipped
 *   %I2C_M_IGNORE_NAK: treat NACK from client as ACK
 *   %I2C_M_REV_DIR_ADDR: toggles the Rd/Wr bit
 *   %I2C_M_STOP: force a STOP condition after the message
 *
 * @len: Number of data bytes in @buf being read from or written to the I2C
 *   slave address. For read transactions where %I2C_M_RECV_LEN is set, the
 *   caller guarantees that this buffer can hold up to %I2C_SMBUS_BLOCK_MAX
 *   bytes in addition to the initial length byte sent by the slave (plus,
 *   if used, the SMBus PEC); and this value will be incremented by the number
 *   of block data bytes received.
 *
 * @buf: The buffer into which data is read, or from which it's written.
 *
 * An i2c_msg is the low level representation of one segment of an I2C
 * transaction.  It is visible to drivers in the @i2c_transfer() procedure,
 * to userspace from i2c-dev, and to I2C adapter drivers through the
 * @i2c_adapter.@master_xfer() method.
 *
 * Except when I2C "protocol mangling" is used, all I2C adapters implement
 * the standard rules for I2C transactions.  Each transaction begins with a
 * START.  That is followed by the slave address, and a bit encoding read
 * versus write.  Then follow all the data bytes, possibly including a byte
 * with SMBus PEC.  The transfer terminates with a NAK, or when all those
 * bytes have been transferred and ACKed.  If this is the last message in a
 * group, it is followed by a STOP.  Otherwise it is followed by the next
 * @i2c_msg transaction segment, beginning with a (repeated) START.
 *
 * Alternatively, when the adapter supports %I2C_FUNC_PROTOCOL_MANGLING then
 * passing certain @flags may have changed those standard protocol behaviors.
 * Those flags are only for use with broken/nonconforming slaves, and with
 * adapters which are known to support the specific mangling options they need.
 */
struct i2c_msg {
  __u16 addr;
  __u16 flags;
#define I2C_M_RD 0x0001         /* guaranteed to be 0x0001! */
#define I2C_M_TEN 0x0010        /* use only if I2C_FUNC_10BIT_ADDR */
#define I2C_M_DMA_SAFE 0x0200   /* use only in kernel space */
#define I2C_M_RECV_LEN 0x0400   /* use only if I2C_FUNC_SMBUS_READ_BLOCK_DATA */
#define I2C_M_NO_RD_ACK 0x0800  /* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK 0x1000 /* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR 0x2000 /* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART 0x4000      /* use only if I2C_FUNC_NOSTART */
#define I2C_M_STOP 0x8000         /* use only if I2C_FUNC_PROTOCOL_MANGLING */
  __u16 len;
  __u8 *buf;
};
