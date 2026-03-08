(define (stream boxel-streams)
  
  ;; Push stream: superseded by pick-and-place for occluder relocation (#53).
  ;; Kept commented out for reference.
  ;; (:stream sample-push-config
  ;;   :inputs (?obj ?b_from)
  ;;   :domain (and (is_object ?obj) (occluder_at ?obj ?b_from))
  ;;   :outputs (?b_to ?q_start ?q_end ?traj)
  ;;   :certified (and
  ;;     (Boxel ?b_to)
  ;;     (Config ?q_start)
  ;;     (Config ?q_end)
  ;;     (Trajectory ?traj)
  ;;     (push_solution ?obj ?b_from ?b_to ?q_start ?q_end ?traj)
  ;;     (config_for_boxel ?q_start ?b_from))
  ;; )
  
  ;; Sample grasp poses for an object  
  (:stream sample-grasp
    :inputs (?o)
    :domain (Obj ?o)
    :outputs (?g)
    :certified (and (Grasp ?g) (valid_grasp ?o ?g))
  )
  
  ;; Plan motion between configurations
  (:stream plan-motion
    :inputs (?q1 ?q2)
    :domain (and (Config ?q1) (Config ?q2))
    :outputs (?t)
    :certified (and (Trajectory ?t) (motion ?q1 ?q2 ?t))
  )
  
  ;; Compute IK solution for picking
  (:stream compute-kin
    :inputs (?o ?b ?g)
    :domain (and (Obj ?o) (Boxel ?b) (valid_grasp ?o ?g))
    :outputs (?q)
    :certified (and (Config ?q) (kin_solution ?o ?b ?g ?q) (config_for_boxel ?q ?b))
  )
)
