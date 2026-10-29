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
