project('dx_stream_dx_msgconvl', 'cpp', version : '1.0.0', license : 'LGPL', default_options: ['cpp_std=c++14'])

opencv_dep = dependency('opencv4', required: true)
json_glib_dep = dependency('json-glib-1.0', version : '>= 1.0')
# Protobuf backend is optional: JSON (the default payload-type) must keep
# building on a machine that has no protobuf installed.
protobuf_opt = get_option('protobuf')
protobuf_dep = dependency('protobuf', required: protobuf_opt)

gst_dep = dependency('gstreamer-1.0', version : '>=1.16.3',
    required : true, fallback : ['gstreamer', 'gst_dep'])

dx_stream_dep = dependency('gstdxstream')
dxrt_dep = declare_dependency(link_args: ['-ldxrt'])

protoc = find_program('protoc', required: protobuf_opt)

with_protobuf = protobuf_dep.found() and protoc.found()

lib_sources = ['dx_msgconvl.cpp', 'dx_msgconvl_priv.cpp']
lib_deps = [gst_dep, opencv_dep, json_glib_dep, dx_stream_dep, dxrt_dep]
lib_args = []

if with_protobuf
    proto_gen = custom_target('dx_frame_pb',
        input : 'proto/dx_frame.proto',
        output : ['dx_frame.pb.cc', 'dx_frame.pb.h'],
        command : [protoc,
                   '--proto_path=@CURRENT_SOURCE_DIR@/proto',
                   '--cpp_out=@OUTDIR@',
                   '@INPUT@'])
    lib_sources += proto_gen
    lib_deps += protobuf_dep
    lib_args += '-DDX_MSGCONVL_WITH_PROTOBUF'
    message('dx_msgconvl: protobuf payload backend ENABLED')
else
    message('dx_msgconvl: protobuf payload backend DISABLED ' +
            '(payload-type=protobuf will be rejected at runtime; ' +
            'install libprotobuf-dev + protobuf-compiler and reconfigure ' +
            'with -Dprotobuf=enabled to build it)')
endif

dx_msgconvl_lib = shared_library('dx_msgconvl',
    lib_sources,
    cpp_args : lib_args,
    dependencies : lib_deps,
    install: true,
    install_dir: get_option('datadir') / 'gstdxstream' / 'lib'
)

# Host unit test (no NPU): protobuf serialization round-trip.
# Only meaningful when the protobuf backend is actually built.
if with_protobuf
    test_protobuf_convert = executable('test_protobuf_convert',
        [
        'tests/test_protobuf_convert.cpp',
        'dx_msgconvl_priv.cpp',
        proto_gen
        ],
        cpp_args : lib_args,
        dependencies : lib_deps,
        install: false
    )
    test('protobuf_convert', test_protobuf_convert)
endif
