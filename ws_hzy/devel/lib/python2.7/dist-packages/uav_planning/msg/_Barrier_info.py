# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from uav_planning/Barrier_info.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Barrier_info(genpy.Message):
  _md5sum = "1a5469db9dded0c3a8fee01955f5cff1"
  _type = "uav_planning/Barrier_info"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float32 t
float32 h
float32 gamma
float32 b
float32 u1
float32 u2
float32 x
float32 y
float32 b_t"""
  __slots__ = ['t','h','gamma','b','u1','u2','x','y','b_t']
  _slot_types = ['float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       t,h,gamma,b,u1,u2,x,y,b_t

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Barrier_info, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.t is None:
        self.t = 0.
      if self.h is None:
        self.h = 0.
      if self.gamma is None:
        self.gamma = 0.
      if self.b is None:
        self.b = 0.
      if self.u1 is None:
        self.u1 = 0.
      if self.u2 is None:
        self.u2 = 0.
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.b_t is None:
        self.b_t = 0.
    else:
      self.t = 0.
      self.h = 0.
      self.gamma = 0.
      self.b = 0.
      self.u1 = 0.
      self.u2 = 0.
      self.x = 0.
      self.y = 0.
      self.b_t = 0.

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
      _x = self
      buff.write(_get_struct_9f().pack(_x.t, _x.h, _x.gamma, _x.b, _x.u1, _x.u2, _x.x, _x.y, _x.b_t))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.t, _x.h, _x.gamma, _x.b, _x.u1, _x.u2, _x.x, _x.y, _x.b_t,) = _get_struct_9f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_9f().pack(_x.t, _x.h, _x.gamma, _x.b, _x.u1, _x.u2, _x.x, _x.y, _x.b_t))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.t, _x.h, _x.gamma, _x.b, _x.u1, _x.u2, _x.x, _x.y, _x.b_t,) = _get_struct_9f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_9f = None
def _get_struct_9f():
    global _struct_9f
    if _struct_9f is None:
        _struct_9f = struct.Struct("<9f")
    return _struct_9f