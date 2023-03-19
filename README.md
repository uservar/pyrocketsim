# pyrocketsim
Python bindings for RocketSim using pybind11

## Clone & Build
```
git clone https://github.com/uservar/pyrocketsim
cd pyrocketsim
cmake -B ./build -DCMAKE_BUILD_TYPE=Release
cmake --build ./build --config Release --parallel
```
The built binaries will either be in ./build/ or ./build/Release/

Note: As a limitation of using pybind, binaries will only work with the specific python version that was used in building them.

## Example Usage

```py
import pyrocketsim as RS


# setup rocketsim arena
tick_rate = 120
arena = RS.Arena(RS.SOCCAR, tick_rate)
print(f"Arena tick rate: {arena.tick_rate}")

# setup ball initial state
ball_state = arena.ball.get_state()
ball_state.pos = RS.Vec(500, 500, 1500)
ball_state.vel = RS.Vec(0, 0, 0.1)  # A ball with 0 vel will not fall
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
arena.step(8)

# some classes have custom __format__ functions
car_state = car.get_state()
print(f"{car_state.last_controls = }")
print(f"{car_state.pos = :.2f}")
print(f"{car_state.rot_mat = :.2f}")
```

If you want an idea of what's included in the bindings you can either read the [source code](https://github.com/uservar/pyrocketsim/blob/main/src/pyrocketsim.cpp) or [pydoc pyrocketsim](https://gist.github.com/uservar/95bdfef383f691181883ddb2615be443).