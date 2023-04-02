# pyrocketsim
Python bindings for RocketSim using pybind11

## Clone
Make sure to clone recursively or git init/update submodules later
```
git clone --recursive https://github.com/uservar/pyrocketsim/
cd pyrocketsim
```

## Build
You need to have a C++20 compiler and CMake >= 3.13 installed and in your path.

- Then, you can install locally using pip:
```
pip install -e .
```
You can set CMAKE_BUILD_PARALLEL_LEVEL environment variable to limit the number of parallel jobs.

- Or, you can build directly:
```
cmake -B ./build -DCMAKE_BUILD_TYPE=Release
cmake --build ./build --config Release --parallel
```
In which case, the built binaries will either be in ./build/ or ./build/Release/

If you want to specify the python version, set PYBIND11_PYTHON_VERSION env variable to "3.x" or use -DPYBIND11_PYTHON_VERSION=3.x when building directly with cmake.

Note: As a limitation of using pybind, the compiled binaries will only work with the specific python version that was used in building them.

## Example Usage

```py
import pyrocketsim as rs

# Initialize RocketSim with a folder to collision meshes
# You can specify a relative or absolute (str / os.path / pathlib) path
# By default, it looks for a relative folder called "collision_meshes"
rs.init()

# setup rocketsim arena
tick_rate = 120
arena = rs.Arena(rs.SOCCAR, tick_rate)
print(f"Arena tick rate: {arena.tick_rate}")

# optional mutators, many of them are not listed here
mutator_config = arena.get_mutator_config()
mutator_config.ball_radius = 50
mutator_config.boost_force = 9000
mutator_config.gravity = rs.Vec(0, -80, -200)
mutator_config.demo_mode = rs.DemoMode.ON_CONTACT
arena.set_mutator_config(mutator_config)

# setup ball initial state
ball_state = arena.ball.get_state()
ball_state.pos = rs.Vec(0, 0, 500)
ball_state.vel = rs.Vec(0, -6000, 0)  # A ball with 0 vel will not fall
arena.ball.set_state(ball_state)
print("Set ball state")

# setup rocketsim cars
for i in range(2):
    team = rs.BLUE if i % 2 else rs.ORANGE
    car = arena.add_car(team, rs.OCTANE)
    car_state = car.get_state()
    car_state.boost = 100
    car_state.pos = rs.Vec(car.id * 200, car.id * 200, 200)
    car_state.vel = rs.Vec(100, 100, 100)
    car_state.ang_vel = rs.Vec(0, 0, 5.5)
    car.set_state(car_state)
    print(f"Car added to team {team} with id {car.id}")

# set car controls
car = arena.get_cars()[0]
car_controls = rs.CarControls(throttle=1, pitch=-1)
car.set_controls(car_controls)

# do any number of steps
arena.step(6)

# some classes have custom __format__ functions
car_state = car.get_state()
print(f"{car_state.last_controls = }")
print(f"{car_state.pos = :.2f}")
print(f"{car_state.rot_mat = :.2f}")
print(f"{car_state.ball_hit_info = :.2f}")

# there's some helper functions
arena.set_goal_score_callback(lambda arena, team: print(f"{team} scored!"))
arena.set_car_bump_callback(
    lambda arena, bumper, victim, is_demo: print(f"{bumper} bumped {victim}!"))
arena.step(100)

old_arena = arena.clone(copy_callbacks=False)
old_arena.write_to_file("arena_file_path")
same_old_arena = rs.Arena.load_from_file("arena_file_path")

arena.reset_to_random_kickoff()
```

If you want an idea of what's included in the bindings you can either read the [source code](src) or [pydoc pyrocketsim](https://gist.github.com/uservar/95bdfef383f691181883ddb2615be443).
