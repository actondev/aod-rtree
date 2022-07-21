catch2_proj = subproject('catch2')

catch2_dep = catch2_proj.get_variable('catch2_dep')

# superliminal: comparing vs mine
nushoin_rtree_proj = subproject('nushoin_rtree')
nushoin_rtree_dep = nushoin_rtree_proj.get_variable('nushoin_rtree_dep')

catch_test_src = files(
  'catch2.cpp', # CATCH_CONFIG_MAIN
  'test_catch.cpp'
)


test_cpp_args = [
    '-Wno-narrowing',
    '-Wno-sign-compare',
    '-DDEBUG',
    # '-DCATCH_CONFIG_ENABLE_BENCHMARKING'
]
test_cpp_args += cpp_args

catch_tests = executable(
  'catch-tests',
  catch_test_src,
  dependencies : [
    catch2_dep,
    aod_rtree_dep,
    nushoin_rtree_dep,
  ],
  cpp_args: test_cpp_args
)

test('catch',
     catch_tests,
     workdir: meson.project_source_root(),
     args: [
       # '[fixme]',
       '-d', 'yes' # test timings
     ]
    )

if get_option('benchmark')
  subdir('benchmark')
endif
