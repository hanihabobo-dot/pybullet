(define (stream boxel-streams)
  
  ;; Compute a push plan: where to push the object and how to get there.
  ;; Input: the object and its current location.
  ;; Output: destination boxel, robot start/end configs, push trajectory.
  ;; Domain uses is_object (static) for both inputs so PDDLStream can bind
  ;; them. The push action precondition (obj_at_boxel ?obj ?b_from) enforces
  ;; that the object is actually at b_from.
  ;; Certifies config_for_boxel so the move action can reach the start config.
  (:stream sample-push-config
    :inputs (?obj ?b_from)
    :domain (and (is_object ?obj) (is_object ?b_from))
    :outputs (?b_to ?q_start ?q_end ?traj)
    :certified (and
      (Boxel ?b_to)
      (Config ?q_start)
      (Config ?q_end)
      (Trajectory ?traj)
      (push_solution ?obj ?b_from ?b_to ?q_start ?q_end ?traj)
      (config_for_boxel ?q_start ?b_from))
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
