library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ov5640_gray_capture is
  generic (
    CAP_PIXELS : natural := 10000  -- 100x100 grayscale = 10,000 bytes
  );
  port (
    pclk        : in  std_logic;
    rst         : in  std_logic;  -- synchronous reset to pclk, active-high

    -- control from system clock domain (hold high to arm capture)
    start_level : in  std_logic;

    -- camera signals (all synchronous to pclk)
    vsync       : in  std_logic;
    href        : in  std_logic;
    data        : in  std_logic_vector(7 downto 0);

    -- write port into image buffer (word writes)
    wr_en       : out std_logic;
    wr_addr     : out unsigned(11 downto 0); -- enough for 0..4095 (needs 0..2499)
    wr_data     : out std_logic_vector(31 downto 0);

    -- status (pclk domain)
    busy        : out std_logic;
    done_toggle : out std_logic
  );
end entity;

architecture rtl of ov5640_gray_capture is
  signal start_ff1, start_ff2 : std_logic := '0';

  signal vsync_d  : std_logic := '0';
  signal href_d   : std_logic := '0';

  signal capturing : std_logic := '0';

  signal pixel_cnt : unsigned(13 downto 0) := (others => '0'); -- up to 16383
  signal word_idx  : unsigned(11 downto 0) := (others => '0'); -- up to 4095
  signal byte_idx  : unsigned(1 downto 0)  := (others => '0'); -- 0..3
  signal pack_reg  : std_logic_vector(31 downto 0) := (others => '0');

  -- For YUV422: alternate bytes, take Y on phase='0', ignore chroma on phase='1'
  signal y_phase : std_logic := '0';

  -- partial-word flush
  signal flush_pending : std_logic := '0';
  signal done_tog      : std_logic := '0';
begin
  done_toggle <= done_tog;

  process(pclk)
    variable pack_next : std_logic_vector(31 downto 0);
  begin
    if rising_edge(pclk) then
      if rst = '1' then
        start_ff1 <= '0';
        start_ff2 <= '0';
        vsync_d <= '0';
        href_d  <= '0';

        capturing <= '0';
        pixel_cnt <= (others => '0');
        word_idx  <= (others => '0');
        byte_idx  <= (others => '0');
        pack_reg  <= (others => '0');
        y_phase   <= '0';

        wr_en   <= '0';
        wr_addr <= (others => '0');
        wr_data <= (others => '0');

        flush_pending <= '0';
        done_tog <= '0';

      else
        -- defaults
        wr_en <= '0';

        -- sync start_level into pclk domain
        start_ff1 <= start_level;
        start_ff2 <= start_ff1;

        -- edge history
        vsync_d <= vsync;
        href_d  <= href;

        -- flush last partial word if needed
        if flush_pending = '1' then
          wr_en   <= '1';
          wr_addr <= word_idx;
          wr_data <= pack_reg;

          flush_pending <= '0';
          done_tog <= not done_tog;
        end if;

        -- Start of frame: VSYNC rising edge arms a new capture if start_level is high
        if (start_ff2 = '1') and (vsync = '1') and (vsync_d = '0') then
          capturing <= '1';
          pixel_cnt <= (others => '0');
          word_idx  <= (others => '0');
          byte_idx  <= (others => '0');
          pack_reg  <= (others => '0');
          y_phase   <= '0';
          flush_pending <= '0';
        end if;

        -- Reset Y/chroma phase at each line start (HREF rising)
        if (href = '1') and (href_d = '0') then
          y_phase <= '0';
        elsif href = '0' then
          y_phase <= '0';
        end if;

        -- Capture pixels
        if (capturing = '1') and (href = '1') and (flush_pending = '0') then
          -- toggle phase each byte during active line
          if y_phase = '0' then
            -- this byte is Y (grayscale)
            pack_next := pack_reg;

            case to_integer(byte_idx) is
              when 0 => pack_next(7 downto 0)   := data;
              when 1 => pack_next(15 downto 8)  := data;
              when 2 => pack_next(23 downto 16) := data;
              when others => pack_next(31 downto 24) := data;
            end case;

            pack_reg <= pack_next;

            -- advance pixel count
            pixel_cnt <= pixel_cnt + 1;

            -- when we filled 4 bytes, write a word
            if byte_idx = "11" then
              wr_en   <= '1';
              wr_addr <= word_idx;
              wr_data <= pack_next;

              word_idx <= word_idx + 1;
              byte_idx <= (others => '0');
            else
              byte_idx <= byte_idx + 1;
            end if;

            -- stop condition
            if pixel_cnt = to_unsigned(CAP_PIXELS - 1, pixel_cnt'length) then
              capturing <= '0';

              -- flush partial word if we didn’t end exactly on a word boundary
              if byte_idx /= "11" then
                flush_pending <= '1';
              else
                done_tog <= not done_tog;
              end if;
            end if;
          end if;

          -- advance phase (Y then chroma then Y then chroma...)
          y_phase <= not y_phase;
        end if;

      end if;
    end if;
  end process;

  busy <= capturing or flush_pending;

end architecture;

