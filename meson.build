project('aod-rtree', 'cpp',
       	default_options: [
	  'cpp_std=c++17',
	],
       )

immer_proj = subproject('immer')
immer_dep = immer_proj.get_variable('immer_dep')

include_dirs = include_directories('src')

sources = files(
  'src/aod/rtree_base.cpp',
)
cpp_args = []
if get_option('test')
  cpp_args += '-DDEBUG'
endif

# disabling asserts
# can be done with meson configure build -Db_ndebug=true
# cpp_args += '-DNDEBUG'

aod_rtree = static_library('aod-rtree',
                sources: sources,
                include_directories:  [
                  include_dirs,
                ],
                dependencies: [immer_dep],
                cpp_args: cpp_args
)

aod_rtree_dep = declare_dependency(
  include_directories: include_dirs,
  dependencies: [immer_dep],
  # performance impact when using linked version?
  # comparing by removing link_with & using sources
  link_with: aod_rtree,
  # sources: sources
)

if get_option('test')
  subdir('test')
endif
