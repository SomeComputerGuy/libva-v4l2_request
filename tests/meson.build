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
