"""autogenerated by genpy from billiards_msgs/LocalizeTableGoal.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import billiards_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class LocalizeTableGoal(genpy.Message):
  _md5sum = "70508e5dfcc8c4be58259ff6bdc4ab36"
  _type = "billiards_msgs/LocalizeTableGoal"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
TableState state

================================================================================
MSG: billiards_msgs/TableState
BallState[] balls

================================================================================
MSG: billiards_msgs/BallState
int8                  id                  # id of the ball
                                          # 0-cue ball, 1-15 for each of the 15 balls
bool                  pocketed            # false if ball is on the table
geometry_msgs/PointStamped   point        # location of each ball
int8                  group_id            # if we want to group the ball

================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

"""
  __slots__ = ['state']
  _slot_types = ['billiards_msgs/TableState']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       state

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LocalizeTableGoal, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.state is None:
        self.state = billiards_msgs.msg.TableState()
    else:
      self.state = billiards_msgs.msg.TableState()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.state.balls)
      buff.write(_struct_I.pack(length))
      for val1 in self.state.balls:
        _x = val1
        buff.write(_struct_bB.pack(_x.id, _x.pocketed))
        _v1 = val1.point
        _v2 = _v1.header
        buff.write(_struct_I.pack(_v2.seq))
        _v3 = _v2.stamp
        _x = _v3
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v2.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v4 = _v1.point
        _x = _v4
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        buff.write(_struct_b.pack(val1.group_id))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.state is None:
        self.state = billiards_msgs.msg.TableState()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.state.balls = []
      for i in range(0, length):
        val1 = billiards_msgs.msg.BallState()
        _x = val1
        start = end
        end += 2
        (_x.id, _x.pocketed,) = _struct_bB.unpack(str[start:end])
        val1.pocketed = bool(val1.pocketed)
        _v5 = val1.point
        _v6 = _v5.header
        start = end
        end += 4
        (_v6.seq,) = _struct_I.unpack(str[start:end])
        _v7 = _v6.stamp
        _x = _v7
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v6.frame_id = str[start:end].decode('utf-8')
        else:
          _v6.frame_id = str[start:end]
        _v8 = _v5.point
        _x = _v8
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 1
        (val1.group_id,) = _struct_b.unpack(str[start:end])
        self.state.balls.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.state.balls)
      buff.write(_struct_I.pack(length))
      for val1 in self.state.balls:
        _x = val1
        buff.write(_struct_bB.pack(_x.id, _x.pocketed))
        _v9 = val1.point
        _v10 = _v9.header
        buff.write(_struct_I.pack(_v10.seq))
        _v11 = _v10.stamp
        _x = _v11
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v10.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v12 = _v9.point
        _x = _v12
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        buff.write(_struct_b.pack(val1.group_id))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.state is None:
        self.state = billiards_msgs.msg.TableState()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.state.balls = []
      for i in range(0, length):
        val1 = billiards_msgs.msg.BallState()
        _x = val1
        start = end
        end += 2
        (_x.id, _x.pocketed,) = _struct_bB.unpack(str[start:end])
        val1.pocketed = bool(val1.pocketed)
        _v13 = val1.point
        _v14 = _v13.header
        start = end
        end += 4
        (_v14.seq,) = _struct_I.unpack(str[start:end])
        _v15 = _v14.stamp
        _x = _v15
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v14.frame_id = str[start:end].decode('utf-8')
        else:
          _v14.frame_id = str[start:end]
        _v16 = _v13.point
        _x = _v16
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 1
        (val1.group_id,) = _struct_b.unpack(str[start:end])
        self.state.balls.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b = struct.Struct("<b")
_struct_2I = struct.Struct("<2I")
_struct_bB = struct.Struct("<bB")
_struct_3d = struct.Struct("<3d")
