(define (stream boxel-streams)
  
  ;; Sample a robot configuration for pushing an occluder aside
  ;; Certifies config_for_boxel so the move action knows the destination
  (:stream sample-push-config
    :inputs (?occ)
    :domain (is_object ?occ)
    :outputs (?q)
    :certified (and (Config ?q) (push_config ?occ ?q) (config_for_boxel ?q ?occ))
  )
  
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
