//...standard class body

class LStream : public std::stringbuf{
protected:
    int sync() {
        UE_LOG(LogTemp, Log, TEXT("%s"), *FString(str().c_str()));
        str("");
        return std::stringbuf::sync();
    }
};
