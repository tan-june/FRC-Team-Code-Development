#include <Bling.h>


Bling::Bling(){

blinkin = new frc::Spark(8);

}

void Bling::BlingRed(){
blinkin -> Set(0.61);
}

void Bling::BlingBlue(){
blinkin -> Set(0.87);
}

void Bling::BlingGreen(){
    blinkin -> Set(0.77);
}
