#include <fstream> 
#include <iostream> 
#include <string.h> 
  
using namespace std; 

string MEZOK[2]={"E00+SR1", "E01+SR1", "E03+SR1"};

string KESZULEKEK[4]={"DVEZ" , "KOZP_OGYD", "DTVA", "OGYD", "7SL87","FVMP";};
  

int main() 
{ 
    int TIPUS[200]={0}; //keszulekek tipusai szamozva itt lesznek
    string MINDEN[200]={0};//keszulekek nevei
    cout << "Adja meg a switchet, majd a hozzá tartozó készülékeket \n"; 
    string filename = "input.txt"; 
    ifstream inputFile(filename); 
    if (!inputFile.is_open()) { 
        cerr << "Error opening file: " << filename << endl; 
        return 1; 
    } 
    string aktualis_mezo; 
    cout << "   A fájl sorai: \n"; 
    int sorszam=0;
    while (getline(inputFile, aktualis_mezo)) { 
        MINDEN[sorszam]=aktualis_mezo;
        cout << aktualis_mezo << endl;   
        sorszam++;  
    } 
    inputFile.close(); 
    //minden cucc beolvasva 


for (int k = 0; k < sorszam; i++) 
{
    if(k==0) //az alállomás neve KISV/DEBD 4 BETUS
    {
        TIPUS[k]=1000;
        continue;
    }

    int e=0;
    for(int i=0;i<3;i++) //MEZOSZAM AZONOSITO
        {
        if(strncmp( MINDEN[k],MEZOK[i])==0)
        //megvan egy MEZO }
        {TIPUS[k]=2;e++;} break; }


    for(int i=0;i<5;i++) //keszulek AZONOSITO
        {   
        if(strncmp(MINDEN[k],KESZULEKEK[i])==0)
        //MEGVAN A KESZULEK
        {TIPUS[k]=3;e++;} break; }

    if(e==0) //Mező neve pl Károlyfalva
    {TIPUS[k]=100;}
    
    if(strncmp(MINDEN[k],"0")==0) //nincs több
    {
        break;
    }
} //tudjuk hogy melyik sor milyen típusú


    // ide ezek alapján a word fgv



    //ide ezek alapján az excelező fgv



  
    return 0; 
}


/*
SORSZAMOK KEZIKONYV:
1-SWITCH
2-MEZO szám
3-KESZULEK itt ezt részletezni!!!
100-mező név
1000-alállomás rövidítésE
*/