#pragma once
// MESSAGE ERL_QUAD_STATES PACKING

#define MAVLINK_MSG_ID_ERL_QUAD_STATES 369


typedef struct __mavlink_erl_quad_states_t {
 uint64_t timestamp; /*<  time since system start (microseconds)*/
 float position[3]; /*<  position*/
 float orientation[4]; /*<  orientation in quaternion*/
 float velocity[3]; /*<  velocity*/
 float angular_velocity[3]; /*<  angular velocity*/
 float controls[4]; /*<  thrust and torques*/
 float controls_scaled[4]; /*<  thrust and torques after battery scaling*/
} mavlink_erl_quad_states_t;

#define MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN 92
#define MAVLINK_MSG_ID_ERL_QUAD_STATES_MIN_LEN 92
#define MAVLINK_MSG_ID_369_LEN 92
#define MAVLINK_MSG_ID_369_MIN_LEN 92

#define MAVLINK_MSG_ID_ERL_QUAD_STATES_CRC 253
#define MAVLINK_MSG_ID_369_CRC 253

#define MAVLINK_MSG_ERL_QUAD_STATES_FIELD_POSITION_LEN 3
#define MAVLINK_MSG_ERL_QUAD_STATES_FIELD_ORIENTATION_LEN 4
#define MAVLINK_MSG_ERL_QUAD_STATES_FIELD_VELOCITY_LEN 3
#define MAVLINK_MSG_ERL_QUAD_STATES_FIELD_ANGULAR_VELOCITY_LEN 3
#define MAVLINK_MSG_ERL_QUAD_STATES_FIELD_CONTROLS_LEN 4
#define MAVLINK_MSG_ERL_QUAD_STATES_FIELD_CONTROLS_SCALED_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ERL_QUAD_STATES { \
    369, \
    "ERL_QUAD_STATES", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_erl_quad_states_t, timestamp) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_erl_quad_states_t, position) }, \
         { "orientation", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_erl_quad_states_t, orientation) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_erl_quad_states_t, velocity) }, \
         { "angular_velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_erl_quad_states_t, angular_velocity) }, \
         { "controls", NULL, MAVLINK_TYPE_FLOAT, 4, 60, offsetof(mavlink_erl_quad_states_t, controls) }, \
         { "controls_scaled", NULL, MAVLINK_TYPE_FLOAT, 4, 76, offsetof(mavlink_erl_quad_states_t, controls_scaled) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ERL_QUAD_STATES { \
    "ERL_QUAD_STATES", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_erl_quad_states_t, timestamp) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_erl_quad_states_t, position) }, \
         { "orientation", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_erl_quad_states_t, orientation) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_erl_quad_states_t, velocity) }, \
         { "angular_velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_erl_quad_states_t, angular_velocity) }, \
         { "controls", NULL, MAVLINK_TYPE_FLOAT, 4, 60, offsetof(mavlink_erl_quad_states_t, controls) }, \
         { "controls_scaled", NULL, MAVLINK_TYPE_FLOAT, 4, 76, offsetof(mavlink_erl_quad_states_t, controls_scaled) }, \
         } \
}
#endif

/**
 * @brief Pack a erl_quad_states message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  time since system start (microseconds)
 * @param position  position
 * @param orientation  orientation in quaternion
 * @param velocity  velocity
 * @param angular_velocity  angular velocity
 * @param controls  thrust and torques
 * @param controls_scaled  thrust and torques after battery scaling
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_erl_quad_states_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const float *position, const float *orientation, const float *velocity, const float *angular_velocity, const float *controls, const float *controls_scaled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float_array(buf, 8, position, 3);
    _mav_put_float_array(buf, 20, orientation, 4);
    _mav_put_float_array(buf, 36, velocity, 3);
    _mav_put_float_array(buf, 48, angular_velocity, 3);
    _mav_put_float_array(buf, 60, controls, 4);
    _mav_put_float_array(buf, 76, controls_scaled, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN);
#else
    mavlink_erl_quad_states_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.orientation, orientation, sizeof(float)*4);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*3);
    mav_array_memcpy(packet.controls, controls, sizeof(float)*4);
    mav_array_memcpy(packet.controls_scaled, controls_scaled, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ERL_QUAD_STATES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ERL_QUAD_STATES_MIN_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_CRC);
}

/**
 * @brief Pack a erl_quad_states message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  time since system start (microseconds)
 * @param position  position
 * @param orientation  orientation in quaternion
 * @param velocity  velocity
 * @param angular_velocity  angular velocity
 * @param controls  thrust and torques
 * @param controls_scaled  thrust and torques after battery scaling
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_erl_quad_states_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const float *position,const float *orientation,const float *velocity,const float *angular_velocity,const float *controls,const float *controls_scaled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float_array(buf, 8, position, 3);
    _mav_put_float_array(buf, 20, orientation, 4);
    _mav_put_float_array(buf, 36, velocity, 3);
    _mav_put_float_array(buf, 48, angular_velocity, 3);
    _mav_put_float_array(buf, 60, controls, 4);
    _mav_put_float_array(buf, 76, controls_scaled, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN);
#else
    mavlink_erl_quad_states_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.orientation, orientation, sizeof(float)*4);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*3);
    mav_array_memcpy(packet.controls, controls, sizeof(float)*4);
    mav_array_memcpy(packet.controls_scaled, controls_scaled, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ERL_QUAD_STATES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ERL_QUAD_STATES_MIN_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_CRC);
}

/**
 * @brief Encode a erl_quad_states struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param erl_quad_states C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_erl_quad_states_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_erl_quad_states_t* erl_quad_states)
{
    return mavlink_msg_erl_quad_states_pack(system_id, component_id, msg, erl_quad_states->timestamp, erl_quad_states->position, erl_quad_states->orientation, erl_quad_states->velocity, erl_quad_states->angular_velocity, erl_quad_states->controls, erl_quad_states->controls_scaled);
}

/**
 * @brief Encode a erl_quad_states struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param erl_quad_states C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_erl_quad_states_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_erl_quad_states_t* erl_quad_states)
{
    return mavlink_msg_erl_quad_states_pack_chan(system_id, component_id, chan, msg, erl_quad_states->timestamp, erl_quad_states->position, erl_quad_states->orientation, erl_quad_states->velocity, erl_quad_states->angular_velocity, erl_quad_states->controls, erl_quad_states->controls_scaled);
}

/**
 * @brief Send a erl_quad_states message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  time since system start (microseconds)
 * @param position  position
 * @param orientation  orientation in quaternion
 * @param velocity  velocity
 * @param angular_velocity  angular velocity
 * @param controls  thrust and torques
 * @param controls_scaled  thrust and torques after battery scaling
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_erl_quad_states_send(mavlink_channel_t chan, uint64_t timestamp, const float *position, const float *orientation, const float *velocity, const float *angular_velocity, const float *controls, const float *controls_scaled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float_array(buf, 8, position, 3);
    _mav_put_float_array(buf, 20, orientation, 4);
    _mav_put_float_array(buf, 36, velocity, 3);
    _mav_put_float_array(buf, 48, angular_velocity, 3);
    _mav_put_float_array(buf, 60, controls, 4);
    _mav_put_float_array(buf, 76, controls_scaled, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ERL_QUAD_STATES, buf, MAVLINK_MSG_ID_ERL_QUAD_STATES_MIN_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_CRC);
#else
    mavlink_erl_quad_states_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.orientation, orientation, sizeof(float)*4);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*3);
    mav_array_memcpy(packet.controls, controls, sizeof(float)*4);
    mav_array_memcpy(packet.controls_scaled, controls_scaled, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ERL_QUAD_STATES, (const char *)&packet, MAVLINK_MSG_ID_ERL_QUAD_STATES_MIN_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_CRC);
#endif
}

/**
 * @brief Send a erl_quad_states message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_erl_quad_states_send_struct(mavlink_channel_t chan, const mavlink_erl_quad_states_t* erl_quad_states)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_erl_quad_states_send(chan, erl_quad_states->timestamp, erl_quad_states->position, erl_quad_states->orientation, erl_quad_states->velocity, erl_quad_states->angular_velocity, erl_quad_states->controls, erl_quad_states->controls_scaled);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ERL_QUAD_STATES, (const char *)erl_quad_states, MAVLINK_MSG_ID_ERL_QUAD_STATES_MIN_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_CRC);
#endif
}

#if MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_erl_quad_states_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const float *position, const float *orientation, const float *velocity, const float *angular_velocity, const float *controls, const float *controls_scaled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float_array(buf, 8, position, 3);
    _mav_put_float_array(buf, 20, orientation, 4);
    _mav_put_float_array(buf, 36, velocity, 3);
    _mav_put_float_array(buf, 48, angular_velocity, 3);
    _mav_put_float_array(buf, 60, controls, 4);
    _mav_put_float_array(buf, 76, controls_scaled, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ERL_QUAD_STATES, buf, MAVLINK_MSG_ID_ERL_QUAD_STATES_MIN_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_CRC);
#else
    mavlink_erl_quad_states_t *packet = (mavlink_erl_quad_states_t *)msgbuf;
    packet->timestamp = timestamp;
    mav_array_memcpy(packet->position, position, sizeof(float)*3);
    mav_array_memcpy(packet->orientation, orientation, sizeof(float)*4);
    mav_array_memcpy(packet->velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet->angular_velocity, angular_velocity, sizeof(float)*3);
    mav_array_memcpy(packet->controls, controls, sizeof(float)*4);
    mav_array_memcpy(packet->controls_scaled, controls_scaled, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ERL_QUAD_STATES, (const char *)packet, MAVLINK_MSG_ID_ERL_QUAD_STATES_MIN_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN, MAVLINK_MSG_ID_ERL_QUAD_STATES_CRC);
#endif
}
#endif

#endif

// MESSAGE ERL_QUAD_STATES UNPACKING


/**
 * @brief Get field timestamp from erl_quad_states message
 *
 * @return  time since system start (microseconds)
 */
static inline uint64_t mavlink_msg_erl_quad_states_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field position from erl_quad_states message
 *
 * @return  position
 */
static inline uint16_t mavlink_msg_erl_quad_states_get_position(const mavlink_message_t* msg, float *position)
{
    return _MAV_RETURN_float_array(msg, position, 3,  8);
}

/**
 * @brief Get field orientation from erl_quad_states message
 *
 * @return  orientation in quaternion
 */
static inline uint16_t mavlink_msg_erl_quad_states_get_orientation(const mavlink_message_t* msg, float *orientation)
{
    return _MAV_RETURN_float_array(msg, orientation, 4,  20);
}

/**
 * @brief Get field velocity from erl_quad_states message
 *
 * @return  velocity
 */
static inline uint16_t mavlink_msg_erl_quad_states_get_velocity(const mavlink_message_t* msg, float *velocity)
{
    return _MAV_RETURN_float_array(msg, velocity, 3,  36);
}

/**
 * @brief Get field angular_velocity from erl_quad_states message
 *
 * @return  angular velocity
 */
static inline uint16_t mavlink_msg_erl_quad_states_get_angular_velocity(const mavlink_message_t* msg, float *angular_velocity)
{
    return _MAV_RETURN_float_array(msg, angular_velocity, 3,  48);
}

/**
 * @brief Get field controls from erl_quad_states message
 *
 * @return  thrust and torques
 */
static inline uint16_t mavlink_msg_erl_quad_states_get_controls(const mavlink_message_t* msg, float *controls)
{
    return _MAV_RETURN_float_array(msg, controls, 4,  60);
}

/**
 * @brief Get field controls_scaled from erl_quad_states message
 *
 * @return  thrust and torques after battery scaling
 */
static inline uint16_t mavlink_msg_erl_quad_states_get_controls_scaled(const mavlink_message_t* msg, float *controls_scaled)
{
    return _MAV_RETURN_float_array(msg, controls_scaled, 4,  76);
}

/**
 * @brief Decode a erl_quad_states message into a struct
 *
 * @param msg The message to decode
 * @param erl_quad_states C-struct to decode the message contents into
 */
static inline void mavlink_msg_erl_quad_states_decode(const mavlink_message_t* msg, mavlink_erl_quad_states_t* erl_quad_states)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    erl_quad_states->timestamp = mavlink_msg_erl_quad_states_get_timestamp(msg);
    mavlink_msg_erl_quad_states_get_position(msg, erl_quad_states->position);
    mavlink_msg_erl_quad_states_get_orientation(msg, erl_quad_states->orientation);
    mavlink_msg_erl_quad_states_get_velocity(msg, erl_quad_states->velocity);
    mavlink_msg_erl_quad_states_get_angular_velocity(msg, erl_quad_states->angular_velocity);
    mavlink_msg_erl_quad_states_get_controls(msg, erl_quad_states->controls);
    mavlink_msg_erl_quad_states_get_controls_scaled(msg, erl_quad_states->controls_scaled);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN? msg->len : MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN;
        memset(erl_quad_states, 0, MAVLINK_MSG_ID_ERL_QUAD_STATES_LEN);
    memcpy(erl_quad_states, _MAV_PAYLOAD(msg), len);
#endif
}
