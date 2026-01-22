/**
 * i2c_transfer - execute a single or combined I2C message
 * @adap: Handle to I2C bus
 * @msgs: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Returns negative errno, else the number of messages executed.
 *
 * Note that there is no requirement that each message be sent to
 * the same slave address, although that is the most common model.
 */
int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num) {
  int ret;

  /* REVISIT the fault reporting model here is weak:
   *
   *  - When we get an error after receiving N bytes from a slave,
   *    there is no way to report "N".
   *
   *  - When we get a NAK after transmitting N bytes to a slave,
   *    there is no way to report "N" ... or to let the master
   *    continue executing the rest of this combined message, if
   *    that's the appropriate response.
   *
   *  - When for example "num" is two and we successfully complete
   *    the first message but get an error part way through the
   *    second, it's unclear whether that should be reported as
   *    one (discarding status on the second message) or errno
   *    (discarding status on the first one).
   */
  ret = __i2c_lock_bus_helper(adap);
  if (ret)
    return ret;

  ret = __i2c_transfer(adap, msgs, num);
  i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);

  return ret;
}
