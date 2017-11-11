#include "road.h"

void Road::update(const std::vector<CarState>& other_cars)
{
  _other_cars = other_cars;
}

CarState* Road::nearest_leading_in_lane(Lane lane) const
{
    CarState *nearest_leading = nullptr;
  
    auto cars_in_lane = all_cars_in_lane(lane);

    for (unsigned int i = 0; i < cars_in_lane.size(); ++i)
    {
      if (cars_in_lane[i].is_in_front(_egocar))
      {
        if (nearest_leading == nullptr)
        {
          nearest_leading = &cars_in_lane[i];
        }
        else if (cars_in_lane[i].s < nearest_leading->s)
        {
          nearest_leading = &cars_in_lane[i];
        }
      }
    }

    return nearest_leading;
}

CarState* Road::nearest_behind_in_lane(Lane lane) const
{
  CarState *nearest_behind = nullptr;

  auto cars_in_lane = all_cars_in_lane(lane);

  for (auto other_car : cars_in_lane)
  {
    if (other_car.is_behind(_egocar))
    {
      if (nearest_behind == nullptr)
      {
        nearest_behind = &other_car;
      }
      else if (other_car.s > nearest_behind->s)
      {
        nearest_behind = &other_car;
      }
    }
  }

  return nearest_behind;
}

std::vector<CarState> Road::all_cars_in_lane(Lane lane) const
{
  std::vector<CarState> cars;

  for (auto car : _other_cars)
  {
    if (car.current_lane == lane)
    {
      cars.push_back(car);
    }
  }

  return cars;
}
