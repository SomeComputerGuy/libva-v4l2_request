hevc_rps_test = executable('hevc_rps_test',
                           'hevc_rps_test.c', '../src/hevc_rps.c',
                           include_directories: include_directories('../src'),
                           install: false)

conf_dir = get_option('hevc_conformance_dir')
if conf_dir != ''
  python = import('python').find_installation('python3')
  test('hevc_rps_conformance', python,
       args: [files('hevc_rps_check.py'), hevc_rps_test,
              conf_dir / 'JCT-VC-HEVC_V1.json', conf_dir / 'x'],
       timeout: 3600)
endif

libva_drm_dep = dependency('libva-drm', required: false)
if libva_drm_dep.found()
  executable('vaprobe', 'vaprobe.c', dependencies: [libva_dep, libva_drm_dep],
             install: false)
endif

locking_test = executable('locking_test', 'locking_test.c',
                          include_directories: [root_inc, include_directories('../src')],
                          link_with: driver_lib,
                          dependencies: [libva_dep, libdrm_dep, threads_dep])
test('locking', locking_test, timeout: 30)

# Includes codec_av1.c itself with the rest of the driver stubbed out.
av1_tiles_test = executable('av1_tiles_test', 'av1_tiles_test.c',
                            include_directories: [root_inc, include_directories('../src')],
                            dependencies: [libva_dep])
test('av1_tiles', av1_tiles_test, timeout: 30)

# devpool.c on its own, with ioctl() playing a mem2mem device.
devpool_test = executable('devpool_test', 'devpool_test.c', '../src/devpool.c',
                          include_directories: include_directories('../src'),
                          link_args: ['-Wl,--wrap=ioctl'],
                          dependencies: [threads_dep])
test('devpool', devpool_test, timeout: 30)

# CPU NV15 -> P010/NV12 conversion against a reference packer (bit-exact,
# no read past a row, padded pitches, threads).
pixconv_test = executable('pixconv_test', 'pixconv_test.c', '../src/pixconv.c',
                          include_directories: include_directories('../src'),
                          dependencies: [threads_dep])
test('pixconv', pixconv_test, timeout: 120)

# 10-bit format negotiation: offered surface formats and conversion routes.
fmtsel_test = executable('fmtsel_test', 'fmtsel_test.c', '../src/fmtsel.c',
                         include_directories: include_directories('../src'),
                         dependencies: [libva_dep])
test('fmtsel', fmtsel_test, timeout: 30)
