#include <linux/const.h>
#include <linux/types.h>

#define GPIO_V2_LINES_MAX 64
#define GPIO_MAX_NAME_SIZE 32
#define GPIO_V2_LINE_NUM_ATTRS_MAX 10

/**
 * enum gpio_v2_line_flag - &struct gpio_v2_line_attribute.flags values
 * @GPIO_V2_LINE_FLAG_USED: line is not available for request
 * @GPIO_V2_LINE_FLAG_ACTIVE_LOW: line active state is physical low
 * @GPIO_V2_LINE_FLAG_INPUT: line is an input
 * @GPIO_V2_LINE_FLAG_OUTPUT: line is an output
 * @GPIO_V2_LINE_FLAG_EDGE_RISING: line detects rising (inactive to active)
 * edges
 * @GPIO_V2_LINE_FLAG_EDGE_FALLING: line detects falling (active to
 * inactive) edges
 * @GPIO_V2_LINE_FLAG_OPEN_DRAIN: line is an open drain output
 * @GPIO_V2_LINE_FLAG_OPEN_SOURCE: line is an open source output
 * @GPIO_V2_LINE_FLAG_BIAS_PULL_UP: line has pull-up bias enabled
 * @GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN: line has pull-down bias enabled
 * @GPIO_V2_LINE_FLAG_BIAS_DISABLED: line has bias disabled
 * @GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME: line events contain REALTIME
 * timestamps
 * @GPIO_V2_LINE_FLAG_EVENT_CLOCK_HTE: line events contain timestamps from
 * the hardware timestamping engine (HTE) subsystem
 */
enum gpio_v2_line_flag {
  GPIO_V2_LINE_FLAG_USED = _BITULL(0),
  GPIO_V2_LINE_FLAG_ACTIVE_LOW = _BITULL(1),
  GPIO_V2_LINE_FLAG_INPUT = _BITULL(2),
  GPIO_V2_LINE_FLAG_OUTPUT = _BITULL(3),
  GPIO_V2_LINE_FLAG_EDGE_RISING = _BITULL(4),
  GPIO_V2_LINE_FLAG_EDGE_FALLING = _BITULL(5),
  GPIO_V2_LINE_FLAG_OPEN_DRAIN = _BITULL(6),
  GPIO_V2_LINE_FLAG_OPEN_SOURCE = _BITULL(7),
  GPIO_V2_LINE_FLAG_BIAS_PULL_UP = _BITULL(8),
  GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN = _BITULL(9),
  GPIO_V2_LINE_FLAG_BIAS_DISABLED = _BITULL(10),
  GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME = _BITULL(11),
  GPIO_V2_LINE_FLAG_EVENT_CLOCK_HTE = _BITULL(12),
};

/**
 * struct gpio_v2_line_attribute - a configurable attribute of a line
 * @id: attribute identifier with value from &enum gpio_v2_line_attr_id
 * @padding: reserved for future use and must be zero filled
 * @flags: if id is %GPIO_V2_LINE_ATTR_ID_FLAGS, the flags for the GPIO
 * line, with values from &enum gpio_v2_line_flag, such as
 * %GPIO_V2_LINE_FLAG_ACTIVE_LOW, %GPIO_V2_LINE_FLAG_OUTPUT etc, added
 * together.  This overrides the default flags contained in the &struct
 * gpio_v2_line_config for the associated line.
 * @values: if id is %GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES, a bitmap
 * containing the values to which the lines will be set, with each bit
 * number corresponding to the index into &struct
 * gpio_v2_line_request.offsets
 * @debounce_period_us: if id is %GPIO_V2_LINE_ATTR_ID_DEBOUNCE, the
 * desired debounce period, in microseconds
 */
struct gpio_v2_line_attribute {
  __u32 id;
  __u32 padding;
  union {
    __aligned_u64 flags;
    __aligned_u64 values;
    __u32 debounce_period_us;
  };
};

/**
 * struct gpio_v2_line_config_attribute - a configuration attribute
 * associated with one or more of the requested lines.
 * @attr: the configurable attribute
 * @mask: a bitmap identifying the lines to which the attribute applies,
 * with each bit number corresponding to the index into &struct
 * gpio_v2_line_request.offsets
 */
struct gpio_v2_line_config_attribute {
  struct gpio_v2_line_attribute attr;
  __aligned_u64 mask;
};

/**
 * struct gpio_v2_line_config - Configuration for GPIO lines
 * @flags: flags for the GPIO lines, with values from &enum
 * gpio_v2_line_flag, such as %GPIO_V2_LINE_FLAG_ACTIVE_LOW,
 * %GPIO_V2_LINE_FLAG_OUTPUT etc, added together.  This is the default for
 * all requested lines but may be overridden for particular lines using
 * @attrs.
 * @num_attrs: the number of attributes in @attrs
 * @padding: reserved for future use and must be zero filled
 * @attrs: the configuration attributes associated with the requested
 * lines.  Any attribute should only be associated with a particular line
 * once.  If an attribute is associated with a line multiple times then the
 * first occurrence (i.e. lowest index) has precedence.
 */
struct gpio_v2_line_config {
  __aligned_u64 flags;
  __u32 num_attrs;
  /* Pad to fill implicit padding and reserve space for future use. */
  __u32 padding[5];
  struct gpio_v2_line_config_attribute attrs[GPIO_V2_LINE_NUM_ATTRS_MAX];
};

/**
 * struct gpio_v2_line_request - Information about a request for GPIO lines
 * @offsets: an array of desired lines, specified by offset index for the
 * associated GPIO chip
 * @consumer: a desired consumer label for the selected GPIO lines such as
 * "my-bitbanged-relay"
 * @config: requested configuration for the lines
 * @num_lines: number of lines requested in this request, i.e. the number
 * of valid fields in the %GPIO_V2_LINES_MAX sized arrays, set to 1 to
 * request a single line
 * @event_buffer_size: a suggested minimum number of line events that the
 * kernel should buffer.  This is only relevant if edge detection is
 * enabled in the configuration. Note that this is only a suggested value
 * and the kernel may allocate a larger buffer or cap the size of the
 * buffer. If this field is zero then the buffer size defaults to a minimum
 * of @num_lines * 16.
 * @padding: reserved for future use and must be zero filled
 * @fd: after a successful %GPIO_V2_GET_LINE_IOCTL operation, contains
 * a valid anonymous file descriptor representing the request
 */
struct gpio_v2_line_request {
  __u32 offsets[GPIO_V2_LINES_MAX];
  char consumer[GPIO_MAX_NAME_SIZE];
  struct gpio_v2_line_config config;
  __u32 num_lines;
  __u32 event_buffer_size;
  /* Pad to fill implicit padding and reserve space for future use. */
  __u32 padding[5];
  __s32 fd;
};
