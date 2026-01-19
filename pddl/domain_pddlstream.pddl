;; =============================================================================
;; Semantic Boxel TAMP Domain - PDDLStream Compatible (Untyped, No Derived)
;; =============================================================================
;; 
;; Uses Know-If fluents for partial observability.
;; Object types are encoded as predicates: (Boxel ?x), (Obj ?x), etc.
;; Derived predicates inlined in action preconditions for compatibility.

(define (domain boxel-tamp)
  (:requirements :strips :equality)
  
  (:predicates
    ;; --- Type predicates ---
    (Boxel ?x)
    (Obj ?x)
    (Config ?x)
    (Trajectory ?x)
    (Grasp ?x)
    
    ;; --- Boxel structure ---
    (semantic_zone ?b)
    (is_shadow ?b)
    (is_occluder ?b)
    
    ;; --- Ground truth (actual world state) ---
    (obj_at_boxel ?o ?b)
    
    ;; --- Know-If fluent (do we know the value?) ---
    (obj_at_boxel_KIF ?o ?b)
    
    ;; --- Robot state ---
    (at_config ?q)
    (handempty)
    (holding ?o)
    (obj_pose_known ?o)
    
    ;; --- Stream certified facts ---
    (sensing_config ?b ?q)
    (valid_grasp ?o ?g)
    (motion ?q1 ?q2 ?t)
    (kin_solution ?o ?b ?g ?q)
  )
  
  ;; =========================================================================
  ;; SENSE_BOXEL: Observe a shadow boxel to check for object
  ;; =========================================================================
  ;; OPTIMISTIC: assumes object will be found (replanning handles failures)
  (:action sense_boxel
    :parameters (?o ?b ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?b)
      (Config ?q)
      (is_shadow ?b)
      (at_config ?q)
      (sensing_config ?b ?q)
      (not (obj_at_boxel_KIF ?o ?b))
    )
    :effect (and
      (obj_at_boxel_KIF ?o ?b)
      (obj_at_boxel ?o ?b)
      (obj_pose_known ?o)
    )
  )
  
  ;; =========================================================================
  ;; MOVE: Move robot from one configuration to another
  ;; =========================================================================
  (:action move
    :parameters (?q1 ?q2 ?t)
    :precondition (and
      (Config ?q1)
      (Config ?q2)
      (Trajectory ?t)
      (at_config ?q1)
      (motion ?q1 ?q2 ?t)
    )
    :effect (and
      (at_config ?q2)
      (not (at_config ?q1))
    )
  )
  
  ;; =========================================================================
  ;; PICK: Pick up an object from a boxel
  ;; =========================================================================
  ;; Precondition: know_obj_at_boxel = (KIF and at) inlined
  (:action pick
    :parameters (?o ?b ?g ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?b)
      (Grasp ?g)
      (Config ?q)
      (handempty)
      (at_config ?q)
      (obj_at_boxel_KIF ?o ?b)
      (obj_at_boxel ?o ?b)
      (kin_solution ?o ?b ?g ?q)
    )
    :effect (and
      (holding ?o)
      (not (handempty))
      (not (obj_at_boxel ?o ?b))
    )
  )
  
  ;; =========================================================================
  ;; PLACE: Place an object in a boxel
  ;; =========================================================================
  (:action place
    :parameters (?o ?b ?g ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?b)
      (Grasp ?g)
      (Config ?q)
      (holding ?o)
      (at_config ?q)
      (kin_solution ?o ?b ?g ?q)
    )
    :effect (and
      (handempty)
      (obj_at_boxel ?o ?b)
      (obj_at_boxel_KIF ?o ?b)
      (not (holding ?o))
    )
  )
)
