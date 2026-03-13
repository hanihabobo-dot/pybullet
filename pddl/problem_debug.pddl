;; Auto-generated problem file from PDDLStreamPlanner.export_problem_pddl()
;; Static init state only — stream-certified facts are NOT included.

(define (problem boxel-tamp-debug)
  (:domain boxel-tamp)

  (:objects
    free_006
    free_007
    free_008
    free_009
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
    obj_003
    obj_004
    obj_005
    q_home
    shadow_000
    shadow_001
    shadow_002
    target_2
  )

  (:init
    (Boxel free_006)
    (Boxel free_007)
    (Boxel free_008)
    (Boxel free_009)
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
    (Boxel obj_003)
    (Boxel obj_004)
    (Boxel obj_005)
    (Boxel shadow_000)
    (Boxel shadow_001)
    (Boxel shadow_002)
    (Config q_home)
    (Obj obj_003)
    (Obj obj_004)
    (Obj obj_005)
    (Obj target_2)
    (at_config q_home)
    (blocks_view_at obj_003 obj_003 shadow_000)
    (blocks_view_at obj_004 obj_004 shadow_001)
    (blocks_view_at obj_005 obj_005 shadow_002)
    (handempty)
    (is_free_space free_006)
    (is_free_space free_007)
    (is_free_space free_008)
    (is_free_space free_009)
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
    (is_object obj_003)
    (is_object obj_004)
    (is_object obj_005)
    (is_shadow shadow_000)
    (is_shadow shadow_001)
    (is_shadow shadow_002)
    (obj_at_boxel obj_003 obj_003)
    (obj_at_boxel obj_004 obj_004)
    (obj_at_boxel obj_005 obj_005)
    (obj_at_boxel_KIF obj_003 obj_003)
    (obj_at_boxel_KIF obj_004 obj_004)
    (obj_at_boxel_KIF obj_005 obj_005)
    (obj_at_boxel_KIF target_2 free_006)
    (obj_at_boxel_KIF target_2 free_007)
    (obj_at_boxel_KIF target_2 free_008)
    (obj_at_boxel_KIF target_2 free_009)
    (obj_at_boxel_KIF target_2 free_010)
    (obj_at_boxel_KIF target_2 free_011)
    (obj_at_boxel_KIF target_2 free_012)
    (obj_at_boxel_KIF target_2 free_013)
    (obj_at_boxel_KIF target_2 free_014)
    (obj_at_boxel_KIF target_2 free_015)
    (obj_at_boxel_KIF target_2 free_016)
    (obj_at_boxel_KIF target_2 free_017)
    (obj_at_boxel_KIF target_2 free_018)
    (obj_at_boxel_KIF target_2 free_019)
    (obj_at_boxel_KIF target_2 free_020)
    (obj_at_boxel_KIF target_2 free_021)
    (obj_at_boxel_KIF target_2 free_022)
    (obj_at_boxel_KIF target_2 free_023)
    (obj_at_boxel_KIF target_2 free_024)
    (obj_at_boxel_KIF target_2 free_025)
    (obj_at_boxel_KIF target_2 free_026)
    (obj_at_boxel_KIF target_2 free_027)
    (obj_at_boxel_KIF target_2 free_028)
    (obj_at_boxel_KIF target_2 free_029)
    (obj_at_boxel_KIF target_2 free_030)
    (obj_at_boxel_KIF target_2 free_031)
    (obj_at_boxel_KIF target_2 free_032)
    (obj_at_boxel_KIF target_2 free_033)
    (obj_at_boxel_KIF target_2 free_034)
    (obj_at_boxel_KIF target_2 free_035)
    (obj_at_boxel_KIF target_2 free_036)
    (obj_at_boxel_KIF target_2 free_037)
    (obj_at_boxel_KIF target_2 free_038)
    (obj_at_boxel_KIF target_2 free_039)
    (obj_at_boxel_KIF target_2 free_040)
    (obj_at_boxel_KIF target_2 free_041)
    (obj_at_boxel_KIF target_2 free_042)
    (obj_at_boxel_KIF target_2 free_043)
    (obj_at_boxel_KIF target_2 free_044)
    (obj_at_boxel_KIF target_2 free_045)
    (obj_at_boxel_KIF target_2 free_046)
    (obj_at_boxel_KIF target_2 free_047)
    (obj_at_boxel_KIF target_2 free_048)
    (obj_at_boxel_KIF target_2 free_049)
    (obj_at_boxel_KIF target_2 free_050)
    (obj_at_boxel_KIF target_2 free_051)
    (obj_at_boxel_KIF target_2 free_052)
    (obj_at_boxel_KIF target_2 free_053)
    (obj_at_boxel_KIF target_2 obj_003)
    (obj_at_boxel_KIF target_2 obj_004)
    (obj_at_boxel_KIF target_2 obj_005)
  )

  (:goal (holding target_2))
)
