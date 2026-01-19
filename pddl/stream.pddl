(define (stream boxel-streams)
  
  ;; Sample a robot configuration for sensing a shadow boxel
  (:stream sample-sensing-config
    :inputs (?b)
    :domain (is_shadow ?b)
    :outputs (?q)
    :certified (and (Config ?q) (sensing_config ?b ?q))
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
    :certified (and (Config ?q) (kin_solution ?o ?b ?g ?q))
  )
)
