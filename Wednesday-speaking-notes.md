# Wednesday speaking notes
Luke Anderson · CS 6376 · November 4, 2026

## Five cues for a no-slide update

| Time | Cue |
|---|---|
| 0:00–0:45 | Picture 20 lights changing from a circle to a grid. Both endpoints can be safe while paths cross. |
| 0:45–1:40 | Who makes which decision? One assignment before departure, then one controller per drone. |
| 1:40–2:40 | Explain the two terms in the acceleration command. Describe how the simulation decides everyone has arrived. |
| 2:40–4:05 | Same circle, same grid, different assignment. Explain 36 versus zero and what one violation means. |
| 4:05–5:30 | Describe a head-on encounter, the planned hybrid modes, and how to make the comparison fair. |

## Suggested script
Use the cues above when speaking. At 125–135 words per minute the script is roughly five to six minutes. Round numbers aloud.

My project is about what happens between two pictures in a drone light show. Imagine twenty drones making a circle, and the next picture is a grid. I can draw both pictures with plenty of space between every pair of drones. But if I pick one drone on the right side of the circle and send it to the left side of the grid, its path might cross somebody else's. The question I want to answer is how much of that problem I can solve when I assign destinations, and what I gain by letting drones react to their neighbors during flight.

In my proposal, I said there would be a central controller. One of the comments I got was to think about what that actually means. Does it have to keep telling each drone where to go? What if communication drops? For the version I have running now, the planner makes one decision up front. It gives every drone a spot in the grid. After that, each drone can work out its next move from its own position, velocity and saved target. So if the planner disappears halfway through, the drone already has the information for this transition. I haven't simulated a radio failure. If only half the drones received their targets before departure, that would be a different problem.

The other decision was how much of a drone to model. Right now I'm assuming it stays at one altitude and can follow a horizontal acceleration command. Each drone has an x and y position and an x and y velocity. The controller does two things. The farther I am from my target, the more I accelerate toward it. The faster I'm already moving, the more the controller brakes. Every twentieth of a second I update the velocity, limit its magnitude and use it to update position. That also gives me a better definition of arrival. A drone could fly straight through the correct spot. I count the transition as complete only once everybody is close to their target and moving slowly for half a second.

The first experiment uses one controller and two ways of handing out targets. In the first run I just numbered the drones around the circle and numbered the grid spots row by row. Drone one gets spot one, and so on. All twenty made it, but the closest pair passed about a centimeter apart. With a spacing rule of three quarters of a meter, I counted thirty-six violations. What I mean by one violation is that a particular pair entered the unsafe region. If those two stayed too close for the next ten updates, that was still one event.

Then I kept the same starting circle, the same grid, and even the same braking and acceleration rules. The only change was picking the assignment that minimized the sum of straight-line distances. This time the closest pair stayed about 1.1 meters apart, and the group reached the grid about two and a half seconds sooner. I reran both cases with a smaller time step and got the same violation counts. What surprised me is that I could get such a different outcome before adding any avoidance at all. It also showed me a problem with how I might test avoidance. If I only showed the second case, the new controller could do absolutely nothing and still look successful.

My next step is to keep the assignment fixed and change what the drones do when another one gets close. I'll start with just two drones headed directly toward each other. I want to see whether they actually pass, or whether they both brake and sit there. In course terms, each drone will switch between tracking, avoiding and holding. I'll give avoidance a different entry and exit distance so a small change in separation doesn't immediately switch it back and forth.

After that I'll run both controllers on the same formation changes and compare clearance, violation counts, time and distance. If two drones stay safely apart but never reach their goals, I'll record a timeout. That's the distinction I want the final demonstration to make clear. Keeping the drones apart and getting the whole swarm into formation are two separate requirements, and I need to measure both.
