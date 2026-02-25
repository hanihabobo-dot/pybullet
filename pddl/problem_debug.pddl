;; Auto-generated problem file from PDDLStreamPlanner.export_problem_pddl()
;; Static init state only — stream-certified facts are NOT included.

(define (problem boxel-tamp-debug)
  (:domain boxel-tamp)

  (:objects
    free_010
    free_011
    free_012
    free_013
    free_014
    free_015
    free_016
    free_017
    free_018
    free_019
    free_020
    free_021
    free_022
    free_023
    free_024
    free_025
    free_026
    free_027
    free_028
    free_029
    free_030
    free_031
    free_032
    free_033
    free_034
    free_035
    free_036
    free_037
    free_038
    free_039
    free_040
    free_041
    free_042
    free_043
    free_044
    free_045
    free_046
    free_047
    free_048
    free_049
    free_050
    free_051
    free_052
    free_053
    free_054
    free_055
    free_056
    obj_000
    obj_001
    obj_002
    obj_003
    q_home
    shadow_004
    shadow_005
    shadow_006
    shadow_007
    shadow_008
    shadow_009
    target_1
  )

  (:init
    (Boxel free_010)
    (Boxel free_011)
    (Boxel free_012)
    (Boxel free_013)
    (Boxel free_014)
    (Boxel free_015)
    (Boxel free_016)
    (Boxel free_017)
    (Boxel free_018)
    (Boxel free_019)
    (Boxel free_020)
    (Boxel free_021)
    (Boxel free_022)
    (Boxel free_023)
    (Boxel free_024)
    (Boxel free_025)
    (Boxel free_026)
    (Boxel free_027)
    (Boxel free_028)
    (Boxel free_029)
    (Boxel free_030)
    (Boxel free_031)
    (Boxel free_032)
    (Boxel free_033)
    (Boxel free_034)
    (Boxel free_035)
    (Boxel free_036)
    (Boxel free_037)
    (Boxel free_038)
    (Boxel free_039)
    (Boxel free_040)
    (Boxel free_041)
    (Boxel free_042)
    (Boxel free_043)
    (Boxel free_044)
    (Boxel free_045)
    (Boxel free_046)
    (Boxel free_047)
    (Boxel free_048)
    (Boxel free_049)
    (Boxel free_050)
    (Boxel free_051)
    (Boxel free_052)
    (Boxel free_053)
    (Boxel free_054)
    (Boxel free_055)
    (Boxel free_056)
    (Boxel obj_000)
    (Boxel obj_001)
    (Boxel obj_002)
    (Boxel obj_003)
    (Boxel shadow_004)
    (Boxel shadow_005)
    (Boxel shadow_006)
    (Boxel shadow_007)
    (Boxel shadow_008)
    (Boxel shadow_009)
    (Config q_home)
    (Obj target_1)
    (at_config q_home)
    (casts_shadow obj_000 shadow_004)
    (casts_shadow obj_001 shadow_005)
    (casts_shadow obj_001 shadow_006)
    (casts_shadow obj_002 shadow_007)
    (casts_shadow obj_002 shadow_008)
    (casts_shadow obj_003 shadow_009)
    (handempty)
    (is_free_space free_010)
    (is_free_space free_011)
    (is_free_space free_012)
    (is_free_space free_013)
    (is_free_space free_014)
    (is_free_space free_015)
    (is_free_space free_016)
    (is_free_space free_017)
    (is_free_space free_018)
    (is_free_space free_019)
    (is_free_space free_020)
    (is_free_space free_021)
    (is_free_space free_022)
    (is_free_space free_023)
    (is_free_space free_024)
    (is_free_space free_025)
    (is_free_space free_026)
    (is_free_space free_027)
    (is_free_space free_028)
    (is_free_space free_029)
    (is_free_space free_030)
    (is_free_space free_031)
    (is_free_space free_032)
    (is_free_space free_033)
    (is_free_space free_034)
    (is_free_space free_035)
    (is_free_space free_036)
    (is_free_space free_037)
    (is_free_space free_038)
    (is_free_space free_039)
    (is_free_space free_040)
    (is_free_space free_041)
    (is_free_space free_042)
    (is_free_space free_043)
    (is_free_space free_044)
    (is_free_space free_045)
    (is_free_space free_046)
    (is_free_space free_047)
    (is_free_space free_048)
    (is_free_space free_049)
    (is_free_space free_050)
    (is_free_space free_051)
    (is_free_space free_052)
    (is_free_space free_053)
    (is_free_space free_054)
    (is_free_space free_055)
    (is_free_space free_056)
    (is_occluder obj_000)
    (is_occluder obj_001)
    (is_occluder obj_002)
    (is_occluder obj_003)
    (is_shadow shadow_004)
    (is_shadow shadow_005)
    (is_shadow shadow_006)
    (is_shadow shadow_007)
    (is_shadow shadow_008)
    (is_shadow shadow_009)
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
    (obj_at_boxel_KIF target_1 obj_000)
    (obj_at_boxel_KIF target_1 obj_001)
    (obj_at_boxel_KIF target_1 obj_002)
    (obj_at_boxel_KIF target_1 obj_003)
    (occluder_blocking obj_000)
    (occluder_blocking obj_001)
    (occluder_blocking obj_002)
    (occluder_blocking obj_003)
  )

  (:goal (holding target_1))
)
