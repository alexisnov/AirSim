//...standard class body

#ifndef common_utils_LStream_hpp
#define common_utils_LStream_hpp

class LStream : public std::stringbuf{
protected:
    int sync() {
        UE_LOG(LogTemp, Log, TEXT("%s"), *FString(str().c_str()));
        str("");
        return std::stringbuf::sync();
    }
};

#endif
