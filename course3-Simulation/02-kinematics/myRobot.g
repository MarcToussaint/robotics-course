world {shape: marker, size: [1.0]}

link0 {X:<t(0 0 .5)>, shape: capsule, size: [1. .2]}
link0>joint1 (link0) {Q:<t(0 0 0.5)>}

joint1(link0>joint1) {joint: hingeY}
link1(joint1) {Q:<t(0 0 0.5)>, shape: capsule, size: [1. .17]}
link1>joint2 (link1) {Q:<t(0 0 0.5)>}

joint2(link1>joint2) {joint: hingeY}
link2(joint2) {Q:<t(0 0 0.5)>, shape: capsule, size: [1. .15]}
link2>joint3 (link2) {Q:<t(0 0 0.5)>}

joint3(link2>joint3) {joint: hingeY}
link3(joint3) {Q:<t(0 0 0.5)>, shape: capsule, size: [1. .13]}
end_effector(link3) {Q:<t(0 0 0.5)>, shape: marker, size: [0.5]}

Edit joint1 { q: 0.785 }
Edit joint2 { q: 1.4 }
Edit joint3 { q: -1.4 }
