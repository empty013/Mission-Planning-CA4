(define (domain mydomain2)

;remove requirements that are not needed
(:requirements :durative-actions :typing :fluents :strips :negative-preconditions)

(:types 
robot - sth
waypoint - sth
battery_station - location
valve - inspectable
pump - inspectable
inspectable - location
route - sth
)

; un-comment following line if constants are needed
;(:constants )

(:predicates
(robotLocation ?waypoint)
(connects ?route - route ?waypoint1 - waypoint ?waypoint2 - waypoint)
(isAtWaypoint ?location - location ?waypoint - waypoint)
(checked ?inspectable)
(pictureTaken ?pump)
(inspected ?valve)
(batteryCharged)
)


(:functions
(distance ?from - waypoint ?to - waypoint)
(robotSpeed)
;(battery_required ?from - waypoint ?to - waypoint)
; (totalDistance)
; (totalTime)
; (battery_charge)
; (battery_recharge_per_time)
; (battery_drain_MOVE)
; (battery_drain_INSPECT)
; (battery_drain_PICTURE)
; (duration_INSPECT)
; (duration_PICTURE)
; (duration_CHARGE)
)

(:durative-action MOVE
    :parameters (?r0 - route ?fromWaypoint - waypoint ?toWaypoint - waypoint)
    :duration (= ?duration (/ (distance ?fromWaypoint ?toWaypoint) (robotSpeed)))
    :condition (and 
        (at start (and 
            (connects ?r0 ?fromWaypoint ?toWaypoint)
            (robotLocation ?fromWaypoint)
            (batteryCharged)
            )
        )
    )
    :effect (and
        (at start (not (robotLocation ?fromWaypoint)))
        (at end (robotLocation ?toWaypoint))
    )
)


; (:durative-action MOVE
;     :parameters (?route0 - route ?fromWaypoint - waypoint ?toWaypoint - waypoint)
;     :duration (= ?duration (/ (distance ?fromWaypoint ?toWaypoint) (robotSpeed)))
;     :condition (and 
;         (at start (and 
;                     ;(connects ?route0 ?fromWaypoint ?toWaypoint)
;                     ;(robotLocation ?fromWaypoint)
;                     (batteryCharged)
;                     ; (>= (battery_charge) (* (battery_drain_MOVE) (/ (distance ?fromWaypoint ?toWaypoint) (robotSpeed))))
;         ))
;         (at end (and 
;                     (robotLocation ?toWaypoint)
;         ))
;     )
;     :effect (and 
;         ;(decrease (battery_charge) (* (battery_drain_MOVE) #t))
;         ;(increase (totalTime) (* #t 1.0))
;         ;(increase (totalDistance) (* #t (robotSpeed)))
;         ;(at start (and
;          ;           (not (robotLocation ?fromWaypoint))
;         ;))
;         (at end (and 
;                     ; (assign (battery_charge) (- (battery_charge) (* (battery_drain_MOVE) (/ (distance ?fromWaypoint ?toWaypoint) (robotSpeed)))))
;                     ;(robotLocation ?toWaypoint)
;             (batteryCharged)
;         ))
;     )
; )

; (:durative-action MOVE_TESTS
;     :parameters (?w1 - waypoint ?w2 - waypoint)
;     :duration (= ?duration 1)
;     :condition (at start (robotLocation ?w1))
    
;     :effect (at start (robotLocation ?w2))
; )


(:durative-action CHARGE
    :parameters (?currentWaypoint - waypoint ?battery_station0 - battery_station)
    :duration (= ?duration 10)
    :condition (at start (and 
                    (isAtWaypoint ?battery_station0 ?currentWaypoint)
                    (robotLocation ?currentWaypoint)
        ))
    :effect (at end (batteryCharged))
)


; (:durative-action CHARGE
;     :parameters (?currentWaypoint - waypoint ?battery_station0 - battery_station)
;     ; :duration (>= ?duration 0)
;     :duration (= ?duration (duration_CHARGE))
;     :condition (and 
;         (at start (and 
;                     (isAtWaypoint ?battery_station0 ?currentWaypoint)
;                     (robotLocation ?currentWaypoint)
;         ))
;         (over all (robotLocation ?currentWaypoint))
;         (at end (and 
;                     (robotLocation ?currentWaypoint)
;         ))
;     )
;     :effect (and
;         (at end 
;         (batteryCharged)
;         ;(increase (battery_charge) (* (battery_recharge_per_time) #t))
;         ;(increase (totalTime) (* #t 1.0))
;     ))
; )


(:durative-action INSPECT
    :parameters (?valve0 - valve ?waypoint0 - waypoint)
    :duration (= ?duration 5)
    :condition (at start (and 
                    (batteryCharged)
                    (robotLocation ?waypoint0)
                    (isAtWaypoint ?valve0 ?waypoint0)
        ))
    
    :effect (at end (checked ?valve0))
)


; (:durative-action INSPECT
;     :parameters (?valve0 - valve ?waypoint0 - waypoint)
;     :duration (= ?duration (duration_INSPECT))
;     :condition (and 
;         (at start (and 
;                     (batteryCharged)
;                     (robotLocation ?waypoint0)
;                     (isAtWaypoint ?valve0 ?waypoint0)
;                     ; (not (checked ?valve0))
;                     (batteryCharged)
;                     ; (>= (battery_charge) (* (battery_drain_INSPECT) (duration_INSPECT)))
;         ))
;         (over all (robotLocation ?waypoint0))
;         (at end (robotLocation ?waypoint0))
;     )
;     :effect (and 
;         ;(decrease (battery_charge) (* #t (battery_drain_INSPECT)))
;         ;(increase (totalTime) (* #t 1.0))
;         (at end (and 
;                     (inspected ?valve0)
;                     (checked ?valve0)
;         ))
;     )
; )


(:durative-action PICTURE
    :parameters (?pump0 - pump ?waypoint0 - waypoint)
    :duration (= ?duration 10)
    :condition (at start (and 
                    (robotLocation ?waypoint0)
                    (isAtWaypoint ?pump0 ?waypoint0)
        ))
    :effect (at end (checked ?pump0))
)


; (:durative-action PICTURE
;     :parameters (?pump0 - pump ?waypoint0 - waypoint)
;     :duration (= ?duration 10)
;     :condition (and 
;         (at start (and 
;                     (robotLocation ?waypoint0)
;                     (isAtWaypoint ?pump0 ?waypoint0)
;                     ; (not (checked ?pump0))
;                     ; (batteryCharged)
;                     ; (>= (battery_charge) (* (battery_drain_PICTURE) (duration_PICTURE)))
;         ))
;         ; (over all (robotLocation ?waypoint0))
;         ; (at end (robotLocation ?waypoint0))
;     )
;     :effect (and 
;         ;(decrease (battery_charge) (* #t (battery_drain_PICTURE)))
;         ;(increase (totalTime) (* #t 1.0))
;         (at end (and 
;                     ; (checked ?pump0)
;                     (pictureTaken ?pump0)
;         ))
;     )
; )

)
