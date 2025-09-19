#include <iostream>
#include <vector>
#include <string>

class Book{
private:
  std::string title;// title 
  std::string author;// author
  int year;// year
  int cost;// cost 

public:
  Book(std::string t, std::string a, int y, int c) : title(t), author(a), year(y), cost(c){}//constructor

  //get title function
  std::string getTitle() const{
    return title;
  }
  //get author function
  std::string getAuthor() const{
  return author;
  }
  //get year function
  int getYear() const{
    return year;
  }
  //get cost function
  int getCost() const{
    return cost;
  }
  //print function
  void printInfo() const {
    std::cout << "Title: " << title << "\n";
    std::cout << "Author: " << author << "\n"; 
    std::cout << "Year: " << year << "\n";
    std::cout << "Cost: " << cost << "\n";
  }
};
class Library{
private:
  std::vector<Book> books;//vector/array books
public: 
  //print books in library
  void printInventory() const{
    std::cout << "Library Inventory: \n";
    for (const Book& book : books){
      book.printInfo();
    }
  }  //adding books to library
  void addBook(const std::string& title, const std::string& author, int year, int cost){
    Book newBook(title, author, year, cost);
    books.push_back(newBook);
  }
};

int main(){
  //library object
  Library mylibrary;
  mylibrary.addBook("1984", "George Orwell", 1949, 5);//library add book
  mylibrary.addBook("Red Rising", "Pierce Brown", 2014, 20);////library add book
  mylibrary.addBook("The Martian", "Andy Weir", 2011, 20);//library add book

  //print full library
  mylibrary.printInventory(); 
}