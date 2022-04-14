# superliminal rtree in github with some minor additions
nushoin_rtree_proj = subproject('nushoin_rtree')
nushoin_rtree_dep = nushoin_rtree_proj.get_variable('nushoin_rtree_dep')

mdds_proj = subproject('mdds')
mdds_dep = mdds_proj.get_variable('mdds_dep')

benchmark_proj = subproject('google-benchmark')
benchmark_dep = benchmark_proj.get_variable('google_benchmark_main_dep')
# google_benchmark_main_dep
# google_benchmark_dep

benchmarks = executable(
  'benchmarks',
  sources: ['benchmark.cpp'],
  dependencies : [
    benchmark_dep,
    aod_rtree_dep,
    nushoin_rtree_dep, # superliminal
    mdds_dep,
  ],
  cpp_args: [
    '-Wno-narrowing',
    '-Wno-sign-compare'
    # '-DCATCH_CONFIG_ENABLE_BENCHMARKING'
  ]
)

benchmark('benchmark',
          benchmarks,
          args: [
            # '--benchmark_filter=init',
            # '--benchmark_filter=copy',
            '--benchmark_filter=search',
          ],
          workdir: meson.project_source_root(),
          timeout: 60,
    )
