FILE(REMOVE_RECURSE
  "srv_gen"
  "src/billiards_planner/srv"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/billiards_planner/PlanOneShot.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
