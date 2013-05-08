FILE(REMOVE_RECURSE
  "srv_gen"
  "src/billiards_planner/srv"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/billiards_planner/srv/__init__.py"
  "src/billiards_planner/srv/_PlanOneShot.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
