(define (problem boxel-sensing-problem)
  (:domain boxel-tamp)
  
  (:objects
    shadow_000 - boxel
    shadow_001 - boxel
    shadow_002 - boxel
    shadow_003 - boxel
    shadow_004 - boxel
    shadow_005 - boxel
    obj_006 - boxel
    obj_007 - boxel
    obj_008 - boxel
    obj_009 - boxel
    free_010 - boxel
    free_011 - boxel
    free_012 - boxel
    free_013 - boxel
    free_014 - boxel
    free_015 - boxel
    free_016 - boxel
    free_017 - boxel
    free_018 - boxel
    free_019 - boxel
    free_020 - boxel
    free_021 - boxel
    free_022 - boxel
    free_023 - boxel
    free_024 - boxel
    free_025 - boxel
    free_026 - boxel
    free_027 - boxel
    free_028 - boxel
    free_029 - boxel
    free_030 - boxel
    free_031 - boxel
    free_032 - boxel
    free_033 - boxel
    free_034 - boxel
    free_035 - boxel
    free_036 - boxel
    free_037 - boxel
    free_038 - boxel
    free_039 - boxel
    free_040 - boxel
    free_041 - boxel
    free_042 - boxel
    free_043 - boxel
    free_044 - boxel
    free_045 - boxel
    free_046 - boxel
    free_047 - boxel
    free_048 - boxel
    free_049 - boxel
    free_050 - boxel
    free_051 - boxel
    free_052 - boxel
    free_053 - boxel
    free_054 - boxel
    free_055 - boxel
    free_056 - boxel
    target_1 - obj
    q_home - config
  )
  
  (:init
    (at_config q_home)
    (handempty)
    (is_object_boxel obj_006)
    (is_object_boxel obj_007)
    (is_object_boxel obj_008)
    (is_object_boxel obj_009)
    (is_shadow shadow_000)
    (is_shadow shadow_001)
    (is_shadow shadow_002)
    (is_shadow shadow_003)
    (is_shadow shadow_004)
    (is_shadow shadow_005)
    (obj_at_boxel_KIF target_1 free_010)
    (obj_at_boxel_KIF target_1 free_011)
    (obj_at_boxel_KIF target_1 free_012)
    (obj_at_boxel_KIF target_1 free_013)
    (obj_at_boxel_KIF target_1 free_014)
    (obj_at_boxel_KIF target_1 free_015)
    (obj_at_boxel_KIF target_1 free_016)
    (obj_at_boxel_KIF target_1 free_017)
    (obj_at_boxel_KIF target_1 free_018)
    (obj_at_boxel_KIF target_1 free_019)
    (obj_at_boxel_KIF target_1 free_020)
    (obj_at_boxel_KIF target_1 free_021)
    (obj_at_boxel_KIF target_1 free_022)
    (obj_at_boxel_KIF target_1 free_023)
    (obj_at_boxel_KIF target_1 free_024)
    (obj_at_boxel_KIF target_1 free_025)
    (obj_at_boxel_KIF target_1 free_026)
    (obj_at_boxel_KIF target_1 free_027)
    (obj_at_boxel_KIF target_1 free_028)
    (obj_at_boxel_KIF target_1 free_029)
    (obj_at_boxel_KIF target_1 free_030)
    (obj_at_boxel_KIF target_1 free_031)
    (obj_at_boxel_KIF target_1 free_032)
    (obj_at_boxel_KIF target_1 free_033)
    (obj_at_boxel_KIF target_1 free_034)
    (obj_at_boxel_KIF target_1 free_035)
    (obj_at_boxel_KIF target_1 free_036)
    (obj_at_boxel_KIF target_1 free_037)
    (obj_at_boxel_KIF target_1 free_038)
    (obj_at_boxel_KIF target_1 free_039)
    (obj_at_boxel_KIF target_1 free_040)
    (obj_at_boxel_KIF target_1 free_041)
    (obj_at_boxel_KIF target_1 free_042)
    (obj_at_boxel_KIF target_1 free_043)
    (obj_at_boxel_KIF target_1 free_044)
    (obj_at_boxel_KIF target_1 free_045)
    (obj_at_boxel_KIF target_1 free_046)
    (obj_at_boxel_KIF target_1 free_047)
    (obj_at_boxel_KIF target_1 free_048)
    (obj_at_boxel_KIF target_1 free_049)
    (obj_at_boxel_KIF target_1 free_050)
    (obj_at_boxel_KIF target_1 free_051)
    (obj_at_boxel_KIF target_1 free_052)
    (obj_at_boxel_KIF target_1 free_053)
    (obj_at_boxel_KIF target_1 free_054)
    (obj_at_boxel_KIF target_1 free_055)
    (obj_at_boxel_KIF target_1 free_056)
    (obj_at_boxel_KIF target_1 obj_006)
    (obj_at_boxel_KIF target_1 obj_007)
    (obj_at_boxel_KIF target_1 obj_008)
    (obj_at_boxel_KIF target_1 obj_009)
    (obj_graspable target_1)
    (occludes obj_006 shadow_000)
    (occludes obj_007 shadow_001)
    (occludes obj_007 shadow_002)
    (occludes obj_008 shadow_003)
    (occludes obj_008 shadow_004)
    (occludes obj_009 shadow_005)
    (semantic_zone free_010)
    (semantic_zone free_011)
    (semantic_zone free_012)
    (semantic_zone free_013)
    (semantic_zone free_014)
    (semantic_zone free_015)
    (semantic_zone free_016)
    (semantic_zone free_017)
    (semantic_zone free_018)
    (semantic_zone free_019)
    (semantic_zone free_020)
    (semantic_zone free_021)
    (semantic_zone free_022)
    (semantic_zone free_023)
    (semantic_zone free_024)
    (semantic_zone free_025)
    (semantic_zone free_026)
    (semantic_zone free_027)
    (semantic_zone free_028)
    (semantic_zone free_029)
    (semantic_zone free_030)
    (semantic_zone free_031)
    (semantic_zone free_032)
    (semantic_zone free_033)
    (semantic_zone free_034)
    (semantic_zone free_035)
    (semantic_zone free_036)
    (semantic_zone free_037)
    (semantic_zone free_038)
    (semantic_zone free_039)
    (semantic_zone free_040)
    (semantic_zone free_041)
    (semantic_zone free_042)
    (semantic_zone free_043)
    (semantic_zone free_044)
    (semantic_zone free_045)
    (semantic_zone free_046)
    (semantic_zone free_047)
    (semantic_zone free_048)
    (semantic_zone free_049)
    (semantic_zone free_050)
    (semantic_zone free_051)
    (semantic_zone free_052)
    (semantic_zone free_053)
    (semantic_zone free_054)
    (semantic_zone free_055)
    (semantic_zone free_056)
    (semantic_zone obj_006)
    (semantic_zone obj_007)
    (semantic_zone obj_008)
    (semantic_zone obj_009)
    (semantic_zone shadow_000)
    (semantic_zone shadow_001)
    (semantic_zone shadow_002)
    (semantic_zone shadow_003)
    (semantic_zone shadow_004)
    (semantic_zone shadow_005)
  )
  
  (:goal
    (holding target_1)
  )
)