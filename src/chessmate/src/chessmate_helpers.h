#include <string>
#include <cctype> 
#include <iostream>


bool is_square_full(const std::string& fen_string,const std::string& square) {
    char board[64];
    for (int i = 0; i < 64; i++) 
        board[i] = 'E'; 

    const size_t size = fen_string.size();
    size_t iter = 0;
    int index = 0;

    
    for (; (iter < size) and (fen_string[iter] != ' '); iter++) {
        if (fen_string[iter] == '/')
            continue;

        if (isdigit(fen_string[iter]))
            index += (fen_string[iter] - '0'); 

        else {
            board[index] = 'F';
            index++;
        }
    }


    int square_index_in_array = (8 - atoi(&square.at(1))) * 8;

    if (square.at(0) == 'a')
        square_index_in_array += 0;

    if (square.at(0) == 'b')
        square_index_in_array += 1;

    if (square.at(0) == 'c')
        square_index_in_array += 2;

    if (square.at(0) == 'd')
        square_index_in_array += 3;

    if (square.at(0) == 'e')
        square_index_in_array += 4;

    if (square.at(0) == 'f')
        square_index_in_array += 5;

    if (square.at(0) == 'g')
        square_index_in_array += 6;

    if (square.at(0) == 'h')
        square_index_in_array += 7;

    return board[square_index_in_array] == 'F';
}


char get_piece_to_take(const std::string& fen_string,const std::string& square) {
    char board[64];
    for (int i = 0; i < 64; i++) 
        board[i] = 'E'; 

    const size_t size = fen_string.size();
    size_t iter = 0;
    int index = 0;

    
    for (; (iter < size) and (fen_string[iter] != ' '); iter++) {
        if (fen_string[iter] == '/')
            continue;

        if (isdigit(fen_string[iter]))
            index += (fen_string[iter] - '0'); 

        else {
            board[index] = fen_string[iter];
            index++;
        }
    }


    int square_index_in_array = (8 - atoi(&square.at(1))) * 8;

    if (square.at(0) == 'a')
        square_index_in_array += 0;

    if (square.at(0) == 'b')
        square_index_in_array += 1;

    if (square.at(0) == 'c')
        square_index_in_array += 2;

    if (square.at(0) == 'd')
        square_index_in_array += 3;

    if (square.at(0) == 'e')
        square_index_in_array += 4;

    if (square.at(0) == 'f')
        square_index_in_array += 5;

    if (square.at(0) == 'g')
        square_index_in_array += 6;

    if (square.at(0) == 'h')
        square_index_in_array += 7;

    return board[square_index_in_array];
}



