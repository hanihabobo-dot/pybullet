;; =============================================================================
;; Semantic Boxel TAMP Domain - PDDLStream Compatible (Untyped, No Derived)
;; =============================================================================
;; 
;; Models the robot's CAPABILITIES (push, sense, move, pick, place) rather
;; than any specific scenario. Scenario-specific spatial relationships
;; (e.g., which objects block which regions) are expressed as problem-level
;; init facts using generic predicates like blocks_view.
;;
;; Uses Know-If fluents for partial observability.
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
    
    ;; --- Boxel classification ---
    (is_shadow ?b)           ; Region not directly visible from the camera
    (is_object ?b)           ; Physical object (can be pushed, picked)
    (is_free_space ?b)       ; Known empty space
    
    ;; --- Visibility (dynamic) ---
    (blocks_view ?obj ?region) ; Object ?obj blocks the camera's view to ?region
    (view_clear ?region)       ; Camera has clear line of sight to ?region
    
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
    (push_config ?obj ?q)         ; Config ?q can push object ?obj
    (valid_grasp ?o ?g)           ; Grasp ?g valid for object ?o
    (motion ?q1 ?q2 ?t)           ; Trajectory ?t from ?q1 to ?q2
    (kin_solution ?o ?b ?g ?q)    ; Config ?q for picking ?o from ?b with ?g
    (config_for_boxel ?q ?b)      ; Config ?q targets boxel ?b (EE inside ?b)
  )
  
  ;; =========================================================================
  ;; PUSH: Push an object to clear the camera's line of sight to a region
  ;; =========================================================================
  ;; The ?region parameter identifies which viewing corridor to clear.
  ;; Effect models the physical consequence: the object no longer blocks
  ;; the view, and the region becomes observable.
  (:action push
    :parameters (?obj ?region ?q)
    :precondition (and
      (Boxel ?obj)
      (Boxel ?region)
      (Config ?q)
      (is_object ?obj)
      (blocks_view ?obj ?region)
      (at_config ?q)
      (push_config ?obj ?q)
      (handempty)
    )
    :effect (and
      (not (blocks_view ?obj ?region))
      (view_clear ?region)
    )
  )
  
  ;; =========================================================================
  ;; SENSE: Observe a region to check for an object
  ;; =========================================================================
  ;; Requires clear line of sight to the region.
  ;; OPTIMISTIC: Assumes object will be found (replanning handles failures)
  (:action sense
    :parameters (?o ?region ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?region)
      (Config ?q)
      (view_clear ?region)
      (at_config ?q)
      (sensing_config ?region ?q)
      (not (obj_at_boxel_KIF ?o ?region))  ; Only sense if unknown
    )
    :effect (and
      (obj_at_boxel_KIF ?o ?region)        ; Now we know
      (obj_at_boxel ?o ?region)            ; OPTIMISTIC: assume found
      (obj_pose_known ?o)
    )
  )
  
  ;; =========================================================================
  ;; MOVE: Move robot from one configuration to another
  ;; =========================================================================
  ;; ?b is the destination boxel — the stream that produced ?q2 certifies
  ;; that the end-effector at ?q2 is within boxel ?b.
  (:action move
    :parameters (?q1 ?q2 ?b ?t)
    :precondition (and
      (Config ?q1)
      (Config ?q2)
      (Boxel ?b)
      (Trajectory ?t)
      (at_config ?q1)
      (config_for_boxel ?q2 ?b)
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
  ;; Destination must be free space
  (:action place
    :parameters (?o ?b ?g ?q)
    :precondition (and
      (Obj ?o)
      (Boxel ?b)
      (Grasp ?g)
      (Config ?q)
      (holding ?o)
      (at_config ?q)
      (is_free_space ?b)
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
