main()
{
    unsigned char *blarg = (unsigned char*)0x0100000;

    for(;;)
        *blarg = *blarg + 1;
}
