# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: imu_samples.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='imu_samples.proto',
  package='app.udp_comm',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x11imu_samples.proto\x12\x0c\x61pp.udp_comm\"\xce\x01\n\nImuSamples\x12\x1b\n\x13time_of_validity_us\x18\x01 \x01(\x12\x12\x12\n\ntimestamps\x18\x02 \x03(\x12\x12\x14\n\x0c\x61\x63\x63\x65l_x_mpss\x18\x03 \x03(\x02\x12\x14\n\x0c\x61\x63\x63\x65l_y_mpss\x18\x04 \x03(\x02\x12\x14\n\x0c\x61\x63\x63\x65l_z_mpss\x18\x05 \x03(\x02\x12\x12\n\ngyro_x_dps\x18\x06 \x03(\x02\x12\x12\n\ngyro_y_dps\x18\x07 \x03(\x02\x12\x12\n\ngyro_z_dps\x18\x08 \x03(\x02\x12\x11\n\ttemp_degc\x18\t \x03(\x02\x62\x06proto3')
)




_IMUSAMPLES = _descriptor.Descriptor(
  name='ImuSamples',
  full_name='app.udp_comm.ImuSamples',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='time_of_validity_us', full_name='app.udp_comm.ImuSamples.time_of_validity_us', index=0,
      number=1, type=18, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamps', full_name='app.udp_comm.ImuSamples.timestamps', index=1,
      number=2, type=18, cpp_type=2, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accel_x_mpss', full_name='app.udp_comm.ImuSamples.accel_x_mpss', index=2,
      number=3, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accel_y_mpss', full_name='app.udp_comm.ImuSamples.accel_y_mpss', index=3,
      number=4, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accel_z_mpss', full_name='app.udp_comm.ImuSamples.accel_z_mpss', index=4,
      number=5, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyro_x_dps', full_name='app.udp_comm.ImuSamples.gyro_x_dps', index=5,
      number=6, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyro_y_dps', full_name='app.udp_comm.ImuSamples.gyro_y_dps', index=6,
      number=7, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyro_z_dps', full_name='app.udp_comm.ImuSamples.gyro_z_dps', index=7,
      number=8, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='temp_degc', full_name='app.udp_comm.ImuSamples.temp_degc', index=8,
      number=9, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=36,
  serialized_end=242,
)

DESCRIPTOR.message_types_by_name['ImuSamples'] = _IMUSAMPLES
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ImuSamples = _reflection.GeneratedProtocolMessageType('ImuSamples', (_message.Message,), dict(
  DESCRIPTOR = _IMUSAMPLES,
  __module__ = 'imu_samples_pb2'
  # @@protoc_insertion_point(class_scope:app.udp_comm.ImuSamples)
  ))
_sym_db.RegisterMessage(ImuSamples)


# @@protoc_insertion_point(module_scope)
