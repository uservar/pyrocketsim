# pyrocketsim
Python bindings for RocketSim using pybind11

## Build
You need to have a C++20 compiler and [CMake](https://cmake.org/) >= 3.13 installed.

Then, you can install locally using pip:
```
pip install -e .
```

Or, you can build directly:
```
cmake -B ./build -DCMAKE_BUILD_TYPE=Release
cmake --build ./build --config Release --parallel
```
In which case, the built binaries will either be in ./build/ or ./build/Release/


Note: As a limitation of using pybind, the compiled binaries will only work with the specific python version that was used in building them.

## Example Usage

```py
import pyrocketsim as RS

# Initialize RocketSim (loads arena collision meshes, etc.)
RS.init()

# setup rocketsim arena
tick_rate = 120
arena = RS.Arena(RS.SOCCAR, tick_rate)
print(f"Arena tick rate: {arena.tick_rate}")


# setup ball initial state
ball_state = arena.ball.get_state()
ball_state.pos = RS.Vec(0, 0, 500)
ball_state.vel = RS.Vec(0, -6000, 0)  # A ball with 0 vel will not fall
arena.ball.set_state(ball_state)
print("Set ball state")

# setup rocketsim cars
for i in range(2):
    team = RS.BLUE if i % 2 else RS.ORANGE
    car = arena.add_car(team, RS.OCTANE)
    car_state = car.get_state()
    car_state.boost = 100
    car_state.pos = RS.Vec(car.id * 200, car.id * 200, 200)
    car_state.vel = RS.Vec(100, 100, 100)
    car_state.ang_vel = RS.Vec(0, 0, 5.5)
    car.set_state(car_state)
    print(f"Car added to team {team} with id {car.id}")

# set car controls
car = arena.get_cars()[0]
car_controls = RS.CarControls(throttle=1, pitch=-1)
car.set_controls(car_controls)

# do any number of steps
arena.step(6)

# some classes have custom __format__ functions
car_state = car.get_state()
print(f"{car_state.last_controls = }")
print(f"{car_state.pos = :.2f}")
print(f"{car_state.rot_mat = :.2f}")

# there's some helper functions
arena.set_goal_score_call_back(lambda arena, team: print(f"{team} scored!"))
arena.step(100)

old_arena = arena.clone(copy_callbacks=False)
old_arena.write_to_file("arena_file_path")
same_old_arena = RS.Arena.load_from_file("arena_file_path")

arena.reset_to_random_kickoff()
```

If you want an idea of what's included in the bindings you can either read the [source code](src/pyrocketsim.cpp) or [pydoc pyrocketsim](https://gist.github.com/uservar/95bdfef383f691181883ddb2615be443).
