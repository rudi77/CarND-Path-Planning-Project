#include "road.h"

void Road::update(const std::vector<CarState>& other_cars)
{
  _other_cars = other_cars;
}

int Road::nearest_leading_in_lane(Lane lane) const
{
    // pointer to the nearest car in front of the EGOCAR
    CarState *nearest_leading = nullptr;
  
    auto cars_in_lane = all_cars_in_lane(lane);

    std::string car_ids = std::accumulate(std::next(cars_in_lane.begin()), cars_in_lane.end(),
      std::to_string(cars_in_lane[0].id), // start with first element
      [](std::string a, CarState b) {
      return a + ' ' + std::to_string(b.id);
    });

    std::cout << "cars " << car_ids << " in lane " << lane << "" << std::endl;

    for (unsigned int i = 0; i < cars_in_lane.size(); ++i)
    {
      if (cars_in_lane[i].is_in_front(_egocar))
      {
        if (nearest_leading == nullptr)
        {
          nearest_leading = &cars_in_lane[i];
          //std::cout << "nearest leading " << nearest_leading->to_string() << "\negocar: " << _egocar.to_string() << std::endl;
        }
        else if (cars_in_lane[i].s < nearest_leading->s)
        {
          nearest_leading = &cars_in_lane[i];
          //std::cout << "new nearest leading " << nearest_leading->to_string() << "\negocar: " << _egocar.to_string() << std::endl;
        }
      }
    }

    auto id = (nearest_leading != nullptr) ? nearest_leading->id : -1;

    return id;
}

int Road::nearest_behind_in_lane(Lane lane) const
{
  // pointer to the nearest car in front of the EGOCAR
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

  return (nearest_behind != nullptr) ? nearest_behind->id : -1;
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
