MEMORY
{
  UNPLACED_SECTIONS (wx) : ORIGIN = 0x100000000, LENGTH = 0
  RAM (wx) : ORIGIN = 0x20000000, LENGTH = 0x00040000
  FLASH (wx) : ORIGIN = 0x00000000, LENGTH = 0x00100000
}

SECTIONS
{
  __RAM_segment_start__ = 0x20000000;
  __RAM_segment_end__ = 0x20040000;
  __RAM_segment_size__ = 0x00040000;
  __FLASH_segment_start__ = 0x00000000;
  __FLASH_segment_end__ = 0x00100000;
  __FLASH_segment_size__ = 0x00100000;

  __HEAPSIZE__ = 8192;
  __STACKSIZE_PROCESS__ = 0;
  __STACKSIZE__ = 8192;

}

