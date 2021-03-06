#pragma once
#include <vector>
#include "helpers.h"
#include "car.h"

class Road
{
public:
  explicit Road(CarState& egocar) : _egocar(egocar) {}

  const std::vector<CarState>& other_cars() const { return _other_cars; }

  void update(const std::vector<CarState>& other_cars);

  bool is_lane_safe(Lane lane) const;

  const CarState* nearest_leading_in_lane(Lane lane) const;

  const CarState* nearest_behind_in_lane(Lane lane) const;

  std::vector<const CarState*> all_cars_in_lane(Lane lane) const;

private:
  CarState& _egocar;
  std::vector<CarState> _other_cars;
};