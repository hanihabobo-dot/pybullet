;; =============================================================================
;; SEMANTIC BOXEL TAMP DOMAIN
;; =============================================================================
;; Domain for Task and Motion Planning with Semantic Boxels under partial
;; observability. Uses Know-If fluents for belief representation.
;;
;; Key Pattern:
;;   - obj_in_boxel(?o, ?b)     = Ground truth (object physically in boxel)
;;   - obj_in_boxel_KIF(?o, ?b) = Know-If (we know the value, true or false)
;;   - K(in)  = KIF ∧ in        = Known to be IN boxel
;;   - K(¬in) = KIF ∧ ¬in       = Known to NOT be in boxel
;;
;; Sensing uses optimistic planning: assumes target will be found.
;; If not found during execution, executor replans with updated state.
;; =============================================================================

(define (domain boxel-tamp)
  (:requirements :strips :typing :equality)
  
  ;; =========================================================================
  ;; TYPES
  ;; =========================================================================
  (:types
    boxel       ;; Semantic boxel (object, shadow, or free space)
    obj         ;; Manipulable object in the scene
    config      ;; Robot configuration (joint angles)
    trajectory  ;; Motion trajectory between configs
    grasp       ;; Grasp transformation
  )
  
  ;; =========================================================================
  ;; PREDICATES
  ;; =========================================================================
  (:predicates
    ;; --- Boxel Structure ---
    (semantic_zone ?b - boxel)            ;; ?b is a valid boxel
    (is_shadow ?b - boxel)                ;; ?b is an occluded/shadow region
    (is_object_boxel ?b - boxel)          ;; ?b bounds a known object
    (neighbor ?b1 ?b2 - boxel)            ;; ?b1 and ?b2 are adjacent
    (occludes ?occ ?shd - boxel)          ;; object boxel ?occ creates shadow ?shd
    
    ;; --- Ground Truth (actual world state, may be unknown to planner) ---
    (obj_at_boxel ?o - obj ?b - boxel)    ;; Object ?o is physically in boxel ?b
    
    ;; --- Know-If Fluent (epistemic state) ---
    (obj_at_boxel_KIF ?o - obj ?b - boxel) ;; We KNOW whether ?o is in ?b
    
    ;; --- Object State ---
    (obj_pose_known ?o - obj)             ;; We know the exact pose of ?o
    (obj_graspable ?o - obj)              ;; ?o can be grasped (not fixed)
    
    ;; --- Robot State ---
    (at_config ?q - config)               ;; Robot is at configuration ?q
    (handempty)                           ;; Gripper is empty
    (holding ?o - obj)                    ;; Gripper is holding object ?o
    
    ;; --- Stream-Certified Predicates (populated by streams) ---
    (sensing_config ?b - boxel ?q - config)           ;; ?q can observe boxel ?b
    (kin_solution ?o - obj ?b - boxel ?g - grasp ?q - config)  ;; IK solution
    (motion_plan ?q1 ?q2 - config ?t - trajectory)    ;; Collision-free path
    (valid_grasp ?o - obj ?g - grasp)                 ;; ?g is valid grasp for ?o
  )
  
  ;; =========================================================================
  ;; ACTIONS
  ;; =========================================================================
  
  ;; -------------------------------------------------------------------------
  ;; SENSE_BOXEL: Look into a shadow boxel to check if target is there
  ;; -------------------------------------------------------------------------
  ;; Optimistic effect: assumes object WILL be found. If not found during
  ;; execution, the executor updates state and replans.
  (:action sense_boxel
    :parameters (?o - obj ?b - boxel ?q - config)
    :precondition (and
      (is_shadow ?b)                      ;; Only sense shadow (unknown) regions
      (at_config ?q)                      ;; Robot at sensing configuration
      (sensing_config ?b ?q)              ;; This config can observe ?b
      (not (obj_at_boxel_KIF ?o ?b))      ;; Don't know if ?o is in ?b yet
      (obj_graspable ?o)                  ;; Only search for graspable objects
    )
    :effect (and
      (obj_at_boxel_KIF ?o ?b)            ;; Now we KNOW the value
      ;; OPTIMISTIC: Assume object is found (replan if wrong)
      (obj_at_boxel ?o ?b)
      (obj_pose_known ?o)
    )
  )
  
  ;; -------------------------------------------------------------------------
  ;; MOVE: Move robot from one configuration to another
  ;; -------------------------------------------------------------------------
  (:action move
    :parameters (?q1 ?q2 - config ?t - trajectory)
    :precondition (and
      (at_config ?q1)
      (motion_plan ?q1 ?q2 ?t)
    )
    :effect (and
      (at_config ?q2)
      (not (at_config ?q1))
    )
  )
  
  ;; -------------------------------------------------------------------------
  ;; PICK: Pick up an object from a known location
  ;; -------------------------------------------------------------------------
  (:action pick
    :parameters (?o - obj ?b - boxel ?g - grasp ?q - config)
    :precondition (and
      (handempty)
      (at_config ?q)
      (obj_at_boxel_KIF ?o ?b)            ;; Must KNOW whether ?o is in ?b
      (obj_at_boxel ?o ?b)                ;; And ?o must actually be there
      (obj_pose_known ?o)                 ;; Must know exact pose
      (valid_grasp ?o ?g)
      (kin_solution ?o ?b ?g ?q)
    )
    :effect (and
      (holding ?o)
      (not (handempty))
      (not (obj_at_boxel ?o ?b))
    )
  )
  
  ;; -------------------------------------------------------------------------
  ;; PLACE: Place a held object in a boxel
  ;; -------------------------------------------------------------------------
  (:action place
    :parameters (?o - obj ?b - boxel ?g - grasp ?q - config)
    :precondition (and
      (holding ?o)
      (at_config ?q)
      (kin_solution ?o ?b ?g ?q)
    )
    :effect (and
      (handempty)
      (not (holding ?o))
      (obj_at_boxel ?o ?b)
      (obj_at_boxel_KIF ?o ?b)            ;; We placed it, so we know
    )
  )
)
