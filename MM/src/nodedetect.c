/* #include <stdio.h> */
/* #include <stdint.h> */


/* uint8_t array[255] = { */
/*   0b00011000, */
/*   0b01011000, */
/*   0b00011000, */
/*   0b01011000, */
/*   0b00011000, */
/*   0b00011000, */
/*   0b01011000, */
/*   0b00011000, */
/*   0b00001111, */
/*   0b00001111, */
/*   0b00001111, */
/*   0b00000000, */
/*   0b00000000, */
/*   0b00000000, */
/* }; */

uint8_t class[7] =  {
  0b00011000,
  0b11111111,
  0b00011111,
  0b11111000,
  0b00000000,
  0b01100110,
};





enum cla {
  MID,
  FULL,
  RIG,
  LFT,
  NUL,
  TT,
  JPT,
};

enum jun {
  DE, // dead end
  ST, // 
  T0, // normal T
  TR, // |-
  TL, // -|
  CR, // +
  SE, //  start or end 
  SH, // SHIT
  RC, // Right cornor
  LC, // left cornor
};


int cur_cunc_state = 0;
enum cla cun_state[3] = {JPT, JPT, JPT};

enum cla chng_state_list[2] = {JPT, JPT};


int cntbits(int n) {
    int count = 0;
    
    while (n) {
        n = n & (n - 1);
        count++;
    }
    
    return count;
}





float weight(uint8_t inp) {
  float retval = 0;

  for (int i = 0; i < 8; i++) {
    int thing = ((inp & (1 << i)) >> i);

    retval += thing * (3.5 - i);

  }

  return retval;
}

/* int logtwo(uint8_t v) { */

/*   unsigned int r = 0; // r will be lg(v) */
  
/*   while (v >>= 1) // unroll for more speed... */
/*     { */
/*       r++; */
/*     } */
/* } */



enum cla get_thing(uint8_t inp) {

  //  printf("log: %i \t", logtwo(inp));


  if(cntbits(inp ^ class[NUL]) <= 1) {
    //printf("NUL\n");
    return NUL;
  }

  if(cntbits(inp ^ class[FULL]) <= 1) {
    //printf("FULL\n");
    return FULL;
  }




  if(cntbits(inp ^ class[RIG]) <= 1) {
    //printf("RIG\n");
    return RIG;
  }

  if(cntbits(inp ^ class[LFT]) <= 1) {
    //printf("LFT\n");
    return LFT;
  }



  if(cntbits(inp ^ class[MID]) <= 1) {
    // printf("MID\n");
    return MID;
  }



  if(cntbits(inp ^ class[TT]) <= 1) {
    printf("TT\n");
    return TT;
  }

  //  printf("JPT\n");
  return JPT;

  
}






void update_cunstate(enum cla inp) {
  cun_state[cur_cunc_state] = inp;
  
  cur_cunc_state = (cur_cunc_state + 1) % 3;
}


int check_concurrency() {
  if (cun_state[0] == cun_state[1] && cun_state[1] == cun_state[2]) {
    return (cun_state[0]);
  } else {
    return -1;
  }
    
}

void move_two(enum cla next) {
  chng_state_list[1] = chng_state_list[0];
  chng_state_list[0] = next;


}



enum jun detect_junction(){

  if (chng_state_list[0] == MID && chng_state_list[1] == FULL) {
    return CR;
  } 

  if (chng_state_list[0] == NUL && chng_state_list[1] == FULL) {

    return T0;
  } 


  if (chng_state_list[0] == NUL && chng_state_list[1] == RIG) {
    return RC;
  } 


  if (chng_state_list[0] == NUL && chng_state_list[FULL] == LFT) {
    return LC;
  } 


  if (chng_state_list[0] == MID && chng_state_list[1] == RIG) {
    return TR;
  } 


  if (chng_state_list[0] == MID && chng_state_list[1] == LFT) {
    return TL;
  } 

  if (chng_state_list[0] == FULL && chng_state_list[1] == TT) {
    return SE;
  }

  if (chng_state_list[0] == NUL && chng_state_list[1] == MID) {
    return DE;
  }
  

  

  return SH;

  
}


  
/* int main() { */
/*   //  detect_junction(array); */
/*   printf("measure: %f\n", weight(0b11111000)); */
/*   /\* printf("lmeasure: %f\n", lweight(0b11111000)); *\/ */
/*   /\* printf("rmeasure: %f\n", rweight(0b11111000)); *\/ */




/*   uint8_t cur; */
/*   for (int i = 0; i < 15; i++) { */
/*     cur = array[i]; */


/*     update_cunstate(get_thing(cur)); */

/*     int concurrent = check_concurrency(); */

/*     if (concurrent < 0) { */
/*       continue; */
/*     } */

/*     /\* if (concurrent == prev_cunstate) { *\/ */
/*     /\*   continue; *\/ */
/*     /\* } *\/ */

/*     move_two(concurrent); */


    
/*     printf("class : %i\n", detect_junction()); */


    
/*   } */



/*   return 0; */
/* } */
