;; =============================================================================
;; Semantic Boxel TAMP Domain - PDDLStream Compatible (Untyped, No Derived)
;; =============================================================================
;; 
;; Uses Know-If fluents for partial observability.
;; Key insight: Shadows are BLOCKED by their occluders. Must push occluder aside
;; before sensing the shadow region.
;;
;; Object types are encoded as predicates: (Boxel ?x), (Obj ?x), etc.

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
    (is_shadow ?b)           ; This boxel is a shadow region
    (is_occluder ?b)         ; This boxel is an occluder object
    (is_free_space ?b)       ; This boxel is free space
    (casts_shadow ?occ ?shd) ; Occluder ?occ casts shadow ?shd
    
    ;; --- Occluder state ---
    (occluder_blocking ?occ) ; Occluder is in its original position (blocking)
    (occluder_aside ?occ)    ; Occluder has been pushed aside (shadow revealed)
    
    ;; --- Ground truth (actual world state) ---
    (obj_at_boxel ?o ?b)     ; Object ?o is physically at boxel ?b
    
    ;; --- Know-If fluent (do we know the value?) ---
    (obj_at_boxel_KIF ?o ?b) ; We know whether ?o is at ?b (true or false)
    
    ;; --- Robot state ---
    (at_config ?q)
    (handempty)
    (holding ?o)
    (obj_pose_known ?o)
    
    ;; --- Stream certified facts ---
    (sensing_config ?b ?q)        ; Config ?q can sense boxel ?b
    (push_config ?occ ?q)         ; Config ?q can push occluder ?occ
    (valid_grasp ?o ?g)           ; Grasp ?g valid for object ?o
    (motion ?q1 ?q2 ?t)           ; Trajectory ?t from ?q1 to ?q2
    (kin_solution ?o ?b ?g ?q)    ; Config ?q for picking ?o from ?b with ?g
  )
  
  ;; =========================================================================
  ;; PUSH_ASIDE: Move an occluder to reveal its shadow
  ;; =========================================================================
  ;; This must be done BEFORE sensing the shadow region
  (:action push_aside
    :parameters (?occ ?q)
    :precondition (and
      (Boxel ?occ)
      (Config ?q)
      (is_occluder ?occ)
      (occluder_blocking ?occ)
      (at_config ?q)
      (push_config ?occ ?q)
      (handempty)
    )
    :effect (and
      (occluder_aside ?occ)
      (not (occluder_blocking ?occ))
    )
  )
  
  ;; =========================================================================
  ;; SENSE_SHADOW: Observe a shadow boxel to check for object
  ;; =========================================================================
  ;; REQUIRES: The occluder must be pushed aside first!
  ;; OPTIMISTIC: Assumes object will be found (replanning handles failures)
  (:action sense_shadow
    :parameters (?o ?shd ?occ ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?shd)
      (Boxel ?occ)
      (Config ?q)
      (is_shadow ?shd)
      (casts_shadow ?occ ?shd)
      (occluder_aside ?occ)           ; MUST have pushed occluder aside!
      (at_config ?q)
      (sensing_config ?shd ?q)
      (not (obj_at_boxel_KIF ?o ?shd)) ; Only sense if unknown
    )
    :effect (and
      (obj_at_boxel_KIF ?o ?shd)       ; Now we know
      (obj_at_boxel ?o ?shd)           ; OPTIMISTIC: assume found
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
  ;; Must KNOW object is there (KIF=true AND at=true)
  (:action pick
    :parameters (?o ?b ?g ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?b)
      (Grasp ?g)
      (Config ?q)
      (handempty)
      (at_config ?q)
      (obj_at_boxel_KIF ?o ?b)        ; Must know
      (obj_at_boxel ?o ?b)            ; Must be there
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
