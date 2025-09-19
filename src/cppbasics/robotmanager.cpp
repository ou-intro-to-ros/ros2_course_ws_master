/*
# robot inventory management system
# print function
add robot function
robot structure
robot name of the robot
cost of the robot
*/
#include <iostream>
#include <vector>
#include <string>

//struct for the robot
struct Robot
{
  std::string name;//name
  float cost;//cost
};
//function to add robot to the inventory
void addRobot(std::vector<Robot>& inventory, std::string name, float cost)
{
  Robot r;//create robot
  r.name = name;//assign member values
  r.cost = cost;//add to inventory
  inventory.push_back(r);
}
//function to print our inventory
void printInventory(std::vector<Robot> inventory)
{
  int i = inventory.size();
  for (int j = 0; j < i; j++)
  { 
    printf("Robot name: %s \n", inventory[j].name.data());
    printf("Robot value: %.2f \n", inventory[j].cost);
  }

  for (int j = 0; j < i; j++)
  { 
    std::cout << "Robot name: " << inventory[j].name << std::endl;
    std::cout << "Robot cost: " << inventory[j].cost << std::endl;
  }
}
//main function
int main()
{
  //create our robot inventory
  std::vector<Robot> inventory;
  //add robots
  addRobot(inventory, "Pepper", 35000.0f);
  addRobot(inventory, "salt", 10000.0f);
  addRobot(inventory, "Wall-E", 150.0f);
  //show all robots (print inventory)
  printInventory(inventory);
  return 0;
}
