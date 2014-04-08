--
-- gpsdo.vhd
--
-- Copyright (C) 2012  Sylvain Munaut <tnt@246tNt.com>
--
-- This program is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program.  If not, see <http://www.gnu.org/licenses/>.
--
-- vim: ts=4 sw=4
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;


entity gpsdo is
	port (
		-- Clock to discipline
		pps_in		: in  std_logic;
		clk_in		: in  std_logic;

		-- Wishbone interface
		wb_adr_i	: in  std_logic_vector(3 downto 0);

		wb_dat_i	: in  std_logic_vector(31 downto 0);
		wb_dat_o	: out std_logic_vector(31 downto 0);

		wb_we_i		: in  std_logic;
		wb_stb_i	: in  std_logic;
		wb_cyc_i	: in  std_logic;
		wb_ack_o	: out std_logic;
		wb_err_o	: out std_logic;
		wb_int_o	: out std_logic;

		wb_clk_i	: in  std_logic;
		wb_rst_i	: in  std_logic
	);
end gpsdo;


architecture rtl of gpsdo is

	-- Shorter rst/clk
	signal clk				: std_logic;
	signal rst				: std_logic;

	-- Wishbone
	signal wb_acc			: std_logic;
	signal wb_wr			: std_logic;
	signal wb_ack			: std_logic;

	signal wb_rd_csr		: std_logic_vector(31 downto 0);

	-- Snap ctrl & status on WB side
	signal snap_req			: std_logic;
	signal snap_clr			: std_logic;
	signal snap_conf		: std_logic;
	signal snap_rdy			: std_logic;

	signal snap_cnt_val		: std_logic_vector(31 downto 0);

	-- Inter clock capture negotiation
	signal sc_send			: std_logic;
	signal sc_send_ccr		: std_logic_vector(0 to 2);
	signal sc_send_hit		: std_logic;

	signal sc_recv			: std_logic;
	signal sc_recv_ccr		: std_logic_vector(0 to 2);
	signal sc_recv_hit		: std_logic;

	-- Capture status (clk_in side)
	signal cap_pending		: std_logic;
	signal cap_conf			: std_logic;

	-- Main cycle counter
	signal c_cnt_cur		: std_logic_vector(31 downto 0);
	signal c_cnt_rst		: std_logic;

	-- Captured cycle counter
	signal c_cnt_captured	: std_logic_vector(31 downto 0);

	-- PPS processing
	signal pps_polarity		: std_logic;
	signal pps_reg			: std_logic_vector(0 to 2);
	signal pps_edge			: std_logic;

begin

	-- Clock / Reset
	clk <= wb_clk_i;
	rst <= wb_rst_i;


	-- WB Synchronous logic
	-------------------------

	-- Wishbone registers
		-- Useful signals
	wb_acc <= wb_cyc_i and wb_stb_i;
	wb_wr  <= wb_cyc_i and wb_stb_i and wb_we_i;

		-- Read mux
	process (clk)
	begin
		if rising_edge(clk) then
			case wb_adr_i is
				when x"0"	=> wb_dat_o <= wb_rd_csr;
				when x"1"	=> wb_dat_o <= snap_cnt_val;
				when others	=> wb_dat_o <= (others => '0');
			end case;
		end if;
	end process;

		-- CSR
	wb_rd_csr <= pps_polarity & (30 downto 2 => '0') & snap_rdy & '0';

		-- Writes
	process (clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				pps_polarity <= '0';
				snap_req <= '0';
			else
				if (wb_wr = '1') and (wb_adr_i = x"0") then
					pps_polarity <= wb_dat_i(31);
					snap_req <= wb_dat_i(0);
				else
					snap_req <= '0';
				end if;
			end if;
		end if;
	end process;

		-- Clear by reading the register
	process (clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				snap_clr <= '0';
			else
				if (wb_acc = '1') and (wb_we_i = '0') and (wb_adr_i = x"1") then
					snap_clr <= '1';
				else
					snap_clr <= '0';
				end if;
			end if;
		end if;
	end process;

		-- Generate ack
	process (clk)
	begin
		if rising_edge(clk) then
			wb_ack <= wb_acc and not wb_ack;
		end if;
	end process;

	wb_ack_o <= wb_ack;

		-- Can't err ...
	wb_err_o <= '0';

		-- WB interrupts
	wb_int_o <= snap_rdy;

	-- Snap ctrl & status
	process (clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				snap_rdy <= '0';
			else
				if (snap_req = '1') or (snap_clr = '1') then
					snap_rdy <= '0';
				elsif sc_recv_hit = '1' then
					snap_rdy <= '1';
				end if;
			end if;
		end if;
	end process;

	process (clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				snap_cnt_val <= (others => '0');
			elsif sc_recv_hit = '1' then
				snap_cnt_val <= c_cnt_captured;
			end if;
		end if;
	end process;

	process (clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				snap_conf <= '0';
			else
				snap_conf <= sc_recv_hit;
			end if;
		end if;
	end process;


	-- Inter clock domain capture negotiation
	-------------------------------------------

	-- Send requests
	process (clk)
	begin
		if rising_edge(clk) then
			if rst = '1' then
				sc_send <= '0';
			elsif snap_req = '1' then
				sc_send <= not sc_send;
			end if;
		end if;
	end process;

	process (clk_in)
	begin
		if rising_edge(clk_in) then
			sc_send_ccr <= sc_send & sc_send_ccr(0 to 1);
		end if;
	end process;

	process (clk_in)
	begin
		if rising_edge(clk_in) then
			sc_send_hit <= sc_send_ccr(1) xor sc_send_ccr(2);
		end if;
	end process;

	-- Receive confirmations
	process (clk_in, rst)
	begin
		if rst = '1' then	-- Async reset ... doesn't matter much except for SIM
			sc_recv <= '0';
		elsif rising_edge(clk_in) then
			if cap_conf = '1' then
				sc_recv <= not sc_recv;
			end if;
		end if;
	end process;

	process (clk)
	begin
		if rising_edge(clk) then
			sc_recv_ccr <= sc_recv & sc_recv_ccr(0 to 1);
		end if;
	end process;

	process (clk)
	begin
		if rising_edge(clk) then
			sc_recv_hit <= sc_recv_ccr(1) xor sc_recv_ccr(2);
		end if;
	end process;


	-- clk_in Synchronous logic
	-----------------------------

	-- Capture pending & confirmation
	process (clk_in)
	begin
		if rising_edge(clk_in) then
			if cap_conf = '1' then
				cap_pending <= '0';
			elsif sc_send_hit = '1' then
				cap_pending <= '1';
			end if;
		end if;
	end process;

	cap_conf <= c_cnt_rst and cap_pending;

	-- Main counter
	process (clk_in)
	begin
		if rising_edge(clk_in) then
			if c_cnt_rst = '1' then
				c_cnt_cur <= (others => '0');
			else
				c_cnt_cur <= c_cnt_cur + 1;
			end if;
		end if;
	end process;

	c_cnt_rst <= pps_edge;

	-- Captured value
	process (clk_in)
	begin
		if rising_edge(clk_in) then
			if cap_conf = '1' then
				c_cnt_captured <= c_cnt_cur;
			end if;
		end if;
	end process;

	-- PPS input and edge detection
	process (clk_in)
	begin
		if rising_edge(clk_in) then
			pps_reg <= pps_in & pps_reg(0 to 1);
		end if;
	end process;

	process (clk_in)
	begin
		if rising_edge(clk_in) then
			pps_edge <= (pps_reg(1) xor pps_polarity) and not (pps_reg(2) xor pps_polarity);
		end if;
	end process;

end rtl;
