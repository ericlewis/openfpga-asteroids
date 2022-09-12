--
-- A simulation model of Asteroids Deluxe hardware
-- Copyright (c) MikeJ - May 2004
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- You are responsible for any legal issues arising from your use of this code.
--
-- The latest version of this file can be found at: www.fpgaarcade.com
--
-- Email support@fpgaarcade.com
--
-- Revision list
--
-- version 001 initial release
--

    --
    -- Notes :
    --
    -- Button shorts input to ground when pressed
	 -- 
	 -- ToDo:
			-- Model sound effects for thump-thump, ship and saucer fire and saucer warble 
			-- Add player control switching and screen flip for cocktail mode 
			-- General cleanup



library ieee;
  use ieee.std_logic_1164.all;
  use ieee.std_logic_arith.all;
  use ieee.std_logic_unsigned.all;

entity ASTEROIDS_TOP is
  port (
    BUTTON            : in std_logic_vector(7 downto 0); -- active low
    LANG					 : in std_logic_vector(1 downto 0);
	 SHIPS				 : in std_logic;
    AUDIO_OUT         : out   std_logic_vector(7 downto 0);
	 SELF_TEST_SWITCH_L: in		std_logic; 

		dn_addr          	: in std_logic_vector(15 downto 0);
		dn_data         	: in std_logic_vector(7 downto 0);
		dn_wr					: in std_logic;
    
    VIDEO_R_OUT       : out   std_logic_vector(3 downto 0);
    VIDEO_G_OUT       : out   std_logic_vector(3 downto 0);
    VIDEO_B_OUT       : out   std_logic_vector(3 downto 0);

    HSYNC_OUT         : out   std_logic;
    VSYNC_OUT         : out   std_logic;
	 VGA_DE				: out std_logic;
	 VID_HBLANK			: out std_logic;
	 VID_VBLANK			: out std_logic;

    RESET_L           : in    std_logic;

    -- ref clock in
	 clk_6					  	:  in  std_logic;
	 clk_25						:  in  std_logic
    );
end;

architecture RTL of ASTEROIDS_TOP is

	signal RAM_ADDR_A        :   std_logic_vector(18 downto 0);
   signal RAM_ADDR_B        :   std_logic_vector(15 downto 0); -- same as above
   signal RAM_WE_L          :   std_logic;
   signal RAM_ADV_L         :   std_logic;
   signal RAM_OE_L          :   std_logic;
   signal RAM_DO           :  std_logic_vector(31 downto 0);
	signal RAM_DI           :  std_logic_vector(31 downto 0);
	signal ram_we          :   std_logic;
	
  signal reset_dll_h          : std_logic;

  signal delay_count          : std_logic_vector(7 downto 0) := (others => '0');
  signal reset_6_l            : std_logic;
  signal reset_6              : std_logic;

  signal clk_cnt              : std_logic_vector(2 downto 0) := "000";

  signal x_vector             : std_logic_vector(9 downto 0);
  signal y_vector             : std_logic_vector(9 downto 0);
  signal z_vector             : std_logic_vector(3 downto 0);
  signal beam_on              : std_logic;
  signal beam_ena             : std_logic;

  signal ram_addr_int         : std_logic_vector(18 downto 0);
  signal ram_we_l_int         : std_logic;
  signal ram_adv_l_int        : std_logic;
  signal ram_oe_l_int         : std_logic;
  signal ram_dout_oe_l        : std_logic;
  signal ram_dout_oe_l_reg    : std_logic;
  signal ram_dout             : std_logic_vector(31 downto 0);
  signal ram_dout_reg         : std_logic_vector(31 downto 0);
  signal ram_din              : std_logic_vector(31 downto 0);

begin

  --
  -- Note about clocks
  --
  -- (the original uses a 6.048 MHz clock, so 40 / 6  - slightly slower)
  --

  reset_dll_h <= not RESET_L;
  reset_6 <= reset_dll_h;

  p_delay : process(RESET_L, clk_6)
  begin
    if (RESET_L = '0') then
      delay_count <= x"00"; -- longer delay for cpu
      reset_6_l <= '0';
    elsif rising_edge(clk_6) then
      if (delay_count(7 downto 0) = (x"FF")) then
        delay_count <= (x"FF");
        reset_6_l <= '1';
      else
        delay_count <= delay_count + "1";
        reset_6_l <= '0';
      end if;
    end if;
  end process;

  u_asteroids : entity work.ASTEROIDS
    port map (
	   SELF_TEST_SWITCH_L => SELF_TEST_SWITCH_L,
      BUTTON            => BUTTON,
      LANG					=> LANG,
		SHIPS					=> SHIPS,
      AUDIO_OUT         => AUDIO_OUT,

      X_VECTOR          => x_vector,
      Y_VECTOR          => y_vector,
      Z_VECTOR          => z_vector,
      BEAM_ON           => beam_on,
      BEAM_ENA          => beam_ena,
      --
      RESET_6_L         => reset_6_l,
      CLK_6             => clk_6,
		clk_25				=> clk_25,
		dn_addr          	=> dn_addr, 
		dn_data         	=> dn_data,
		dn_wr					=> dn_wr				
      );

  u_DW : entity work.ASTEROIDS_DW
    port map (
      RESET            => reset_6,
		clk_25				=> clk_25,
		clk_6					=> clk_6,

      X_VECTOR         => x_vector,
      Y_VECTOR         => y_vector,
      Z_VECTOR         => z_vector,

      BEAM_ON          => beam_on,
      BEAM_ENA         => beam_ena,

      VIDEO_R_OUT      => VIDEO_R_OUT,
      VIDEO_G_OUT      => VIDEO_G_OUT,
      VIDEO_B_OUT      => VIDEO_B_OUT,
      HSYNC_OUT        => HSYNC_OUT,
      VSYNC_OUT        => VSYNC_OUT,
		VID_DE				=> VGA_DE,
		VID_HBLANK		=>	VID_HBLANK,
		VID_VBLANK		=>	VID_VBLANK

      );


end RTL;
